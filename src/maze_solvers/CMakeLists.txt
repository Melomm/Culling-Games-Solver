cmake_minimum_required(VERSION 3.10)
project(maze_solvers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# Part 1
add_executable(solve_part1 solve_part1.cpp)
ament_target_dependencies(solve_part1 rclcpp cg_interfaces)

# Part 2
add_executable(solve_part2 solve_part2.cpp)
ament_target_dependencies(solve_part2 rclcpp cg_interfaces)

install(TARGETS solve_part1 solve_part2 DESTINATION lib/${PROJECT_NAME})

ament_package()