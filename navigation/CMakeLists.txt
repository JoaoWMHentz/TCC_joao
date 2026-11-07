cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# Exploração por fronteiras: o executável importa frontiers.py da mesma pasta
install(PROGRAMS scripts/frontier_explorer.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/frontiers.py DESTINATION lib/${PROJECT_NAME})

ament_package()
