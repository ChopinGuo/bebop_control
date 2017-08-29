cmake_minimum_required(VERSION 2.8.3)
project(bebop_control)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  roscpp
  rospy
  image_transport
  bebop_msgs
  nav_msgs
  tf
)



find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
find_package(class_loader)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITION})

#catkin_python_setup()

set(GUI_SOURCE_FILES   
    src/bebop_control_gui.cpp
    src/bebop_command.cpp
    src/main.cpp
    src/round_thread.cpp
)

set(GUI_HEADER_FILES   
    include/bebop_control_gui.h
    include/bebop_command.h
    include/round_thread.h
)

set(GUI_UI_FILES  
    ui/bebop_control_gui.ui
)

# *.qrc
set(GUI_RESOURCE_FILES        
)

set(bebop_control_INCLUDE_DIRECTORIES
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
  DEPENDS system_lib
)


file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})
include_directories(${bebop_control_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

# build!
add_executable(bebop_gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
target_link_libraries(bebop_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
