#include <ros/package.h>
#include <ros/ros.h>
#include <uav_audio/PlayAudio.h>

#ifdef UAV_AUDIO_WITH_TTS
#include <ros/callback_queue.h>
#include <sherpa-onnx/c-api/c-api.h>
#include <uav_audio/Say.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sys/mman.h>
#include <vector>
#endif

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
constexpr char kDefaultAudioDevice[] =
    "pulse:DEVICE=alsa_output.usb-Generic_USB2.0_Device_20170726905923-01.iec958-stereo";
std::string audio_directory;
std::string audio_device;
std::mutex playback_mutex;
std::mutex state_mutex;
std::condition_variable wake_waiter;
pid_t child_pid = -1;
std::thread waiter;

template <typename Response>
bool reject(Response& response, const std::string& message) {
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

template <typename Response>
bool startPlayback(int file, const std::string& name, bool preempt, Response& response) {
  // Serialize requests even while waiting for the previous player to exit.
  std::lock_guard<std::mutex> playback_lock(playback_mutex);
  std::unique_lock<std::mutex> lock(state_mutex);
  if (!preempt && child_pid > 0) {
    return reject(response, "Playback busy");
  }

  // aplay reads the WAV through stdin.
  posix_spawn_file_actions_t actions;
  int error = posix_spawn_file_actions_init(&actions);
  if (error != 0) {
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

bool play(uav_audio::PlayAudio::Request& request,
          uav_audio::PlayAudio::Response& response) {
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
  const bool result = startPlayback(file, name, true, response);
  close(file);
  return result;
}

#ifdef UAV_AUDIO_WITH_TTS
class TextToSpeech {
 public:
  explicit TextToSpeech(float speed) : speed_(speed) {}

  bool load(const std::string& directory) {
    const std::string model = directory + "/model.onnx";
    const std::string tokens = directory + "/tokens.txt";
    const std::string lexicon = directory + "/lexicon.txt";
    const std::string date = directory + "/date.fst";
    const std::string number = directory + "/number.fst";
    for (const auto& path : {model, tokens, lexicon, date, number}) {
      if (!std::ifstream(path).good()) {
        ROS_ERROR_STREAM("Cannot read MeloTTS file: " << path);
        return false;
      }
    }

    const std::string rules = date + "," + number;
    SherpaOnnxOfflineTtsConfig config{};
    config.model.vits.model = model.c_str();
    config.model.vits.tokens = tokens.c_str();
    config.model.vits.lexicon = lexicon.c_str();
    config.model.num_threads = 2;
    config.model.provider = "cpu";
    config.rule_fsts = rules.c_str();
    tts_.reset(SherpaOnnxCreateOfflineTts(&config));
    return static_cast<bool>(tts_);
  }

  bool say(uav_audio::Say::Request& request, uav_audio::Say::Response& response) {
    const std::string& text = request.text;
    if (text.size() > 512 || text.find('\0') != std::string::npos ||
        text.find_first_not_of(" \t\r\n") == std::string::npos) {
      return reject(response, "Provide non-empty text of at most 512 UTF-8 bytes");
    }
    std::unique_lock<std::mutex> lock(synthesis_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return reject(response, "TTS busy");
    }

    try {
      const std::unique_ptr<const SherpaOnnxGeneratedAudio,
                            decltype(&SherpaOnnxDestroyOfflineTtsGeneratedAudio)> audio(
          SherpaOnnxOfflineTtsGenerate(tts_.get(), text.c_str(), 0, speed_),
          SherpaOnnxDestroyOfflineTtsGeneratedAudio);
      if (!audio || audio->n <= 0) {
        return reject(response, "MeloTTS generated no audio");
      }
      float peak = 0.0f;
      for (int i = 0; i < audio->n; ++i) {
        const float magnitude = std::abs(audio->samples[i]);
        if (!std::isfinite(magnitude)) {
          return reject(response, "MeloTTS generated an invalid sample");
        }
        peak = std::max(peak, magnitude);
      }
      if (peak == 0.0f) {
        return reject(response, "MeloTTS generated only silence");
      }
      std::vector<float> samples(audio->n);
      std::transform(audio->samples, audio->samples + audio->n, samples.begin(),
                     [peak](float sample) { return sample / peak; });
      std::vector<char> wav(SherpaOnnxWaveFileSize(audio->n));
      SherpaOnnxWriteWaveToBuffer(samples.data(), audio->n, audio->sample_rate, wav.data());

      const int fd = memfd_create("uav_audio_tts", MFD_CLOEXEC);
      if (fd < 0) {
        return reject(response, std::string("Cannot create TTS WAV: ") + std::strerror(errno));
      }
      const std::unique_ptr<FILE, decltype(&fclose)> file(fdopen(fd, "wb"), fclose);
      if (!file) {
        close(fd);
        return reject(response, "Cannot open TTS WAV stream");
      }
      if (fwrite(wav.data(), 1, wav.size(), file.get()) != wav.size() ||
          fseek(file.get(), 0, SEEK_SET) != 0) {
        return reject(response, "Cannot write TTS WAV");
      }
      return startPlayback(fd, "MeloTTS", false, response);
    } catch (const std::exception& error) {
      return reject(response, std::string("MeloTTS failed: ") + error.what());
    }
  }

 private:
  const float speed_;
  std::mutex synthesis_mutex_;
  std::unique_ptr<const SherpaOnnxOfflineTts, decltype(&SherpaOnnxDestroyOfflineTts)> tts_{
      nullptr, SherpaOnnxDestroyOfflineTts};
};
#endif
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "audio_node");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  private_node.param<std::string>("audio_device", audio_device, kDefaultAudioDevice);
  const std::string package_path = ros::package::getPath("uav_audio");
  if (package_path.empty() || audio_device.empty()) {
    ROS_FATAL("Cannot locate uav_audio or audio_device is empty");
    return 1;
  }
  audio_directory = package_path + "/audio";
  const ros::ServiceServer service = node.advertiseService("/audio/play", play);
  ROS_INFO_STREAM("/audio/play ready (device=" << audio_device << ")");
  std::string tts_model_dir;
  private_node.getParam("tts_model_dir", tts_model_dir);
#ifdef UAV_AUDIO_WITH_TTS
  float tts_speed;
  private_node.param("tts_speed", tts_speed, 0.8f);
  if (!std::isfinite(tts_speed) || tts_speed <= 0.0f) {
    ROS_WARN("Invalid tts_speed, using 0.8");
    tts_speed = 0.8f;
  }
  TextToSpeech tts(tts_speed);
  ros::CallbackQueue tts_queue;
  ros::AsyncSpinner tts_spinner(2, &tts_queue);
  ros::NodeHandle tts_node;
  tts_node.setCallbackQueue(&tts_queue);
  ros::ServiceServer say_service;
  if (!tts_model_dir.empty()) {
    if (tts.load(tts_model_dir)) {
      say_service = tts_node.advertiseService("/audio/say", &TextToSpeech::say, &tts);
      tts_spinner.start();
      ROS_INFO("/audio/say ready (MeloTTS zh_en)");
    } else {
      ROS_ERROR("MeloTTS unavailable");
    }
  }
#else
  if (!tts_model_dir.empty()) {
    ROS_ERROR("MeloTTS requires a build with SHERPA_ONNX_ROOT");
  }
#endif
  ros::spin();
#ifdef UAV_AUDIO_WITH_TTS
  tts_spinner.stop();
#endif

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
