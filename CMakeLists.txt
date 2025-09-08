cmake_minimum_required(VERSION 3.21)
project(api_test)

find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PDAL REQUIRED CONFIG)
    
add_library(cvwrapper cvwrapper.cpp)
add_library(exporter exporter.cpp)
add_executable(api_test main.c udp_client.c curl_client.c crc.c)

target_link_libraries(cvwrapper ${OpenCV_LIBS})
target_link_libraries(exporter ${PDAL_LIBRARIES})
target_link_libraries(api_test CURL::libcurl cvwrapper exporter)

