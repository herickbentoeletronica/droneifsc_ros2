cmake_minimum_required(VERSION 3.8)
project(droneifsc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Instala as pastas no diretorio de share do ROS2
install(DIRECTORY 
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
