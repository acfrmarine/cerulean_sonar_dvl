cmake_minimum_required(VERSION 3.1.3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(cerulean_sonar_dvl_driver)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nmea_msgs
        diagnostic_msgs
        cerulean_sonar_dvl_msgs
        )

catkin_package()

add_executable(dvl_nmea_sentence_parser src/dvl_nmea_sentence_parser.cpp)
add_dependencies(dvl_nmea_sentence_parser ${catkin_EXPORTED_TARGETS})
target_include_directories(dvl_nmea_sentence_parser SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(dvl_nmea_sentence_parser ${catkin_LIBRARIES})

add_executable(dvl_to_range src/dvl_nmea_sentence_parser.cpp)
add_dependencies(dvl_to_range ${catkin_EXPORTED_TARGETS})
target_include_directories(dvl_to_range SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(dvl_to_range ${catkin_LIBRARIES})

