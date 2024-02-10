cmake_minimum_required(VERSION 3.12)

project(SDP)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../install)

Set(FETCHCONTENT_QUIET FALSE)
# check if kinova api exists
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/lib/release/libKortexApiCpp.a)
    message(WARNING "kinova_api not found.")
    message("Downloading file...")
    File(DOWNLOAD 
        https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/linux_x86_64_gcc_5.4.zip 
        ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api.zip 
        SHOW_PROGRESS STATUS DOWNLOAD_RESULT)
    list(GET DOWNLOAD_RESULT 0 DOWNLOAD_RESULT_CODE)
    if(NOT DOWNLOAD_RESULT_CODE EQUAL 0)
        message(FATAL_ERROR "Failed downloading! Error: ${DOWNLOAD_RESULT}.")
    endif()
    message("Creating directory...")
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api)
    message("Extracting file...")
    execute_process(COMMAND unzip ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api.zip -d ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api OUTPUT_QUIET)
    message("Removing file...")
    file(REMOVE ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api.zip)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/lib/release/libKortexApiCpp.a)
        message(FATAL_ERROR "Something went wrong with the download and extraction of kinova_api.")
    endif()
endif()

# add subdirectories
add_subdirectory(orocos_kinematics_dynamics/orocos_kdl)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/orocos_kinematics_dynamics/orocos_kdl/src/)

add_compile_definitions(_OS_UNIX)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/client
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/messages
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/client_stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/google
)

add_subdirectory(feedback_pouring)