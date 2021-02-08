cmake_minimum_required(VERSION 3.1.3)

project(cerulean_sonar_dvl_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        actionlib_msgs
        )

add_message_files(FILES
        DVLExtendedData.msg
        )

add_service_files(FILES
        SetFloat32.srv
        )

generate_messages(DEPENDENCIES
        std_msgs
        )

catkin_package(CATKIN_DEPENDS
        message_runtime
        std_msgs
        )

