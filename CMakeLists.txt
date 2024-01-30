cmake_minimum_required(VERSION 3.3)
project(hypermap_rviz_plugin)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(hypermap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)

## This plugin includes Qt widgets, so we must include Qt.
find_package(Qt5 REQUIRED COMPONENTS Widgets)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  include/hypermap_rviz_plugin/hypermapdisplay.h
  src/hypermapdisplay.cpp
  include/hypermap_rviz_plugin/semanticmapdisplay.h
  src/semanticmapdisplay.cpp
)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC include ${Qt5Widgets_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME} hypermap_msgs nav_msgs rviz_common rviz_rendering rviz_default_plugins pluginlib)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(hypermap_msgs nav_msgs rviz_common rviz_rendering rviz_default_plugins pluginlib)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()