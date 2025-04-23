cmake_minimum_required(VERSION 3.5)
project(color_sorting)

find_package(ament_cmake_python REQUIRED)

# install your Python package
ament_python_install_package(${PROJECT_NAME})

# install your console_scripts into install/bin
ament_python_install_entry_points(${PROJECT_NAME})

ament_package()
