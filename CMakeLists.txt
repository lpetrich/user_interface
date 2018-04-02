cmake_minimum_required(VERSION 2.8.3)
project(user_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS 
  scripts/user_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
