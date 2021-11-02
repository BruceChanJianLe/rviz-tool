cmake_minimum_required(VERSION 3.0.2)
project(rviz-tool)

set(LIBRARY_NAME "mbfGoal")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  rviz
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rviz-tool
#  CATKIN_DEPENDS pluginlib roscpp rospy rviz
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

# Avoid keyword definition to avoid conflicts with boost signals. explain etc
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
add_definitions(-DQT_NO_KEYWORDS)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/rviz_tool.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/rviz_tool.hpp
)

# Wrap cpp for QT
if(rviz_QT_VERSION VERSION_LESS "5")
    qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
else()
    qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
endif()

## Add library is needed in order to generate the header file from ui file.
add_library(${LIBRARY_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_MOCS} 
)

target_link_libraries(${LIBRARY_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  ${LIBRARY_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  rviz_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rviz-mbf-goal-tool.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
