# dummy package just saw we can use the package name to find config
# files in launch scripts
cmake_minimum_required (VERSION 2.8.3)
project (sensable_phantom_config)
find_package (catkin REQUIRED)
catkin_package ()
