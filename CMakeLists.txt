cmake_minimum_required(VERSION 3.8)
project(qr_reader)


if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
# set(OpenCV_DIR "/usr/include/opencv4/opencv2")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(qrcode_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp
  image_transport
  cv_bridge
  OpenCV
  vision_msgs
  qrcode_msgs
  sensor_msgs
  pcl_ros
  PCL
  message_filters
  visualization_msgs
)


#ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED 
  src/qr_reader/qrcode_pcl.cpp
  src/qr_reader/qrcode_image.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})




add_executable(qrcode_pcl src/qrcode_pcl_main.cpp)
ament_target_dependencies(qrcode_pcl ${dependencies})
target_link_libraries(qrcode_pcl ${PROJECT_NAME})

add_executable(qrcode_image src/qrcode_image_main.cpp)
ament_target_dependencies(qrcode_image ${dependencies})
target_link_libraries(qrcode_image ${PROJECT_NAME})



install(TARGETS
  ${PROJECT_NAME}
  qrcode_pcl
  qrcode_image
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch params model DESTINATION share/${PROJECT_NAME})

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
