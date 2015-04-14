cmake_minimum_required(VERSION 2.8.3)
project(g4s_bringup)

find_package(catkin REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES g4s_bringup
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#TODO: maybe want some install targets

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
