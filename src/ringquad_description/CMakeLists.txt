cmake_minimum_required(VERSION 3.8)
project(ringquad_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf launch rviz config worlds
  DESTINATION share/${PROJECT_NAME})

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
add_library(ringquad_steering_lock SHARED plugins/SteeringLock.cc)
target_link_libraries(ringquad_steering_lock PRIVATE gz-sim8::gz-sim8)
target_compile_features(ringquad_steering_lock PRIVATE cxx_std_17)
install(TARGETS ringquad_steering_lock DESTINATION lib)
ament_package()
