cmake_minimum_required(VERSION 3.0.2)
project(uav_audio)

find_package(catkin REQUIRED COMPONENTS roscpp roslib message_generation)
find_package(Threads REQUIRED)
add_service_files(FILES PlayAudio.srv)
generate_messages()
catkin_package(CATKIN_DEPENDS roscpp roslib message_runtime)

add_executable(audio_node src/audio_node.cpp)
set_target_properties(audio_node PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)
target_compile_options(audio_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(audio_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(audio_node ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(audio_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS audio_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY audio launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
