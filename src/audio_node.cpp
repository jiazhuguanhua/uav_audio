#include <ros/package.h>
#include <ros/ros.h>
#include <uav_audio/PlayAudio.h>

#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <spawn.h>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char** environ;

namespace {
std::string audio_directory;
std::string audio_device;
std::mutex state_mutex;
std::condition_variable wake_waiter;
pid_t child_pid = -1;
std::thread waiter;

bool reject(uav_audio::PlayAudio::Response& response, const std::string& message) {
  response.success = false;
  response.message = message;
  ROS_WARN_STREAM(message);
  return true;  // Application errors belong in the response, not a transport failure.
}

void waitForPlayback(pid_t pid) {
  std::unique_lock<std::mutex> lock(state_mutex);
  int status = 0;
  for (;;) {
    // Reap and clear the PID under one lock so shutdown cannot kill a reused PID.
    const pid_t result = waitpid(pid, &status, WNOHANG);
    if (result == pid) {
      if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        ROS_INFO("Playback finished");
      } else if (WIFEXITED(status)) {
        ROS_ERROR("aplay exited with code %d", WEXITSTATUS(status));
      } else if (WIFSIGNALED(status)) {
        ROS_WARN("aplay terminated by signal %d", WTERMSIG(status));
      }
      break;
    }
    if (result < 0 && errno != EINTR) {
      ROS_ERROR("waitpid failed: %s", std::strerror(errno));
      break;
    }
    wake_waiter.wait_for(lock, std::chrono::milliseconds(20));
  }
  child_pid = -1;
  wake_waiter.notify_all();
}

bool play(uav_audio::PlayAudio::Request& request,
          uav_audio::PlayAudio::Response& response) {
  std::unique_lock<std::mutex> lock(state_mutex);

  const std::string& name = request.filename;
  if (name.size() <= 4 || name.find_first_of("/\\") != std::string::npos ||
      name.find("..") != std::string::npos || name.find('\0') != std::string::npos ||
      name.compare(name.size() - 4, 4, ".wav") != 0) {
    return reject(response, "Invalid filename: use a plain filename ending in .wav");
  }

  // Reject symlinks and pin the opened file through spawn, avoiding path races.
  const int directory = open(audio_directory.c_str(), O_RDONLY | O_DIRECTORY |
                                                    O_NOFOLLOW | O_CLOEXEC);
  if (directory < 0) {
    return reject(response, std::string("Cannot open audio directory: ") + std::strerror(errno));
  }
  const int file = openat(directory, name.c_str(), O_RDONLY | O_NOFOLLOW |
                                                O_NONBLOCK | O_CLOEXEC);
  const int open_error = errno;
  close(directory);
  if (file < 0) {
    return reject(response, "Cannot open WAV file '" + name + "': " + std::strerror(open_error));
  }
  struct stat info {};
  const int stat_result = fstat(file, &info);
  const int stat_error = errno;
  if (stat_result < 0 || !S_ISREG(info.st_mode)) {
    close(file);
    return reject(response, stat_result < 0
                                ? std::string("Cannot inspect WAV file: ") + std::strerror(stat_error)
                                : "WAV file is not a regular file");
  }

  // aplay reads the validated descriptor as stdin; no shell interprets the filename.
  posix_spawn_file_actions_t actions;
  int error = posix_spawn_file_actions_init(&actions);
  if (error != 0) {
    close(file);
    return reject(response, std::string("Cannot prepare aplay: ") + std::strerror(error));
  }
  error = posix_spawn_file_actions_adddup2(&actions, file, STDIN_FILENO);
  char program[] = "aplay";
  char device_option[] = "-D";
  char type_option[] = "-t";
  char type[] = "wav";
  char* arguments[] = {program, device_option, const_cast<char*>(audio_device.c_str()),
                       type_option, type, nullptr};
  pid_t pid = -1;
  if (error == 0 && child_pid > 0) {
    // Stop only our validated, unreaped child; invalid requests leave it playing.
    if (kill(child_pid, SIGKILL) < 0 && errno != ESRCH) {
      error = errno;
    } else {
      ROS_INFO("Preempting current playback");
      wake_waiter.notify_one();
      // Release the mutex so the waiter can reap the child before replacement.
      wake_waiter.wait(lock, [] { return child_pid < 0; });
    }
  }
  if (error == 0) {
    if (waiter.joinable()) {
      waiter.join();
    }
    error = posix_spawnp(&pid, program, &actions, nullptr, arguments, environ);
  }
  posix_spawn_file_actions_destroy(&actions);
  close(file);
  if (error != 0) {
    return reject(response, std::string("Cannot start aplay: ") + std::strerror(error));
  }
  child_pid = pid;
  try {
    waiter = std::thread(waitForPlayback, pid);
  } catch (const std::exception& exception) {
    kill(pid, SIGKILL);
    while (waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {}
    child_pid = -1;
    return reject(response, std::string("Cannot start playback waiter: ") + exception.what());
  }
  response.success = true;
  response.message = "Playback started";
  ROS_INFO_STREAM("Playback started: " << name << " (device=" << audio_device << ")");
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "audio_node");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  private_node.param<std::string>("audio_device", audio_device, "default");
  const std::string package_path = ros::package::getPath("uav_audio");
  if (package_path.empty() || audio_device.empty()) {
    ROS_FATAL("Cannot locate uav_audio or audio_device is empty");
    return 1;
  }
  audio_directory = package_path + "/audio";
  const ros::ServiceServer service = node.advertiseService("/audio/play", play);
  ROS_INFO_STREAM("/audio/play ready (device=" << audio_device << ")");
  ros::spin();

  // Stop our child before joining, even for very long WAVs. Never leave a child behind.
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (child_pid > 0 && kill(child_pid, SIGKILL) < 0 && errno != ESRCH) {
      ROS_ERROR("Cannot terminate aplay: %s", std::strerror(errno));
    }
  }
  wake_waiter.notify_one();
  if (waiter.joinable()) {
    waiter.join();
  }
  return 0;
}
