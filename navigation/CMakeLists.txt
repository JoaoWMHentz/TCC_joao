cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# Exploração: os executáveis importam frontiers.py / wall_bfs.py / pocket.py
# (e o pocket_explorer importa o wall_bfs_explorer) da mesma pasta
install(PROGRAMS scripts/frontier_explorer.py scripts/wall_bfs_explorer.py
        scripts/pocket_explorer.py
        DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/frontiers.py scripts/wall_bfs.py scripts/pocket.py DESTINATION lib/${PROJECT_NAME})

ament_package()
