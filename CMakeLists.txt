cmake_minimum_required(VERSION 3.10)

project(ExtracustomUserApp LANGUAGES CXX)

add_library(UserApp SHARED
    UserApp.cpp
)

target_compile_features(UserApp PRIVATE cxx_std_17)

target_include_directories(UserApp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

set_target_properties(UserApp PROPERTIES
    OUTPUT_NAME "UserApp"
)
