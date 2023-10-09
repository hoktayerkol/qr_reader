cmake_minimum_required(VERSION 3.8)
project(qr_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
#set(OpenCV_DIR "/usr/include/opencv4/opencv2")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(qrcode_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(depandencies
  rclcpp
  image_transport
  cv_bridge
  vision_msgs
  OpenCV
  qrcode_msgs
  )

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED 
  src/qr_reader/qr_reader.cpp 
  )

ament_target_dependencies(${PROJECT_NAME} ${depandencies})

add_executable(qr_reader_node src/qr_reader_main.cpp)
ament_target_dependencies(qr_reader_node ${dependencies})
target_link_libraries(qr_reader_node ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  qr_reader_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch params include model DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
