cmake_minimum_required(VERSION 3.12)

project(feedback_pouring)

# set(CMAKE_PREFIX_PATH 
#     "../orocos_kinematics_dynamics/orocos_kdl"
# )

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

# get project root directory

get_filename_component(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
message(STATUS "Project root directo
ry: ${PROJECT_ROOT_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../outputs/${PROJECT_NAME}/)

# include all hpp files in include folder
include_directories(
    include
    /home/vicky/sdp/kinova_api/include 
    /home/vicky/sdp/orocos_kinematics_dynamics/kdl_parser
  )



# add all cpp files in src folder
file(GLOB SOURCES "src/*.cpp") 

# make all cpp files in src folder as executable
foreach(source ${SOURCES})
    get_filename_component(name ${source} NAME_WE)
    add_executable(${name} ${source})
    TARGET_LINK_LIBRARIES(${name} 
      boost_filesystem boost_system boost_iostreams
      yaml-cpp kdl_parser orocos_kdl
    #   kinova_mediator
    ../kinova_api/lib/release/libKortexApiCpp.a pthread)
endforeach(source ${SOURCES})