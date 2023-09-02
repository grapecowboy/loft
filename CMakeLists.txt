cmake_minimum_required(VERSION 3.5)
project("bernie")

set(CMAKE_CXX_STANDARD 14)

add_executable(${PROJECT_NAME} "")
target_sources(
    ${PROJECT_NAME} PUBLIC 
        "main.cpp" 
        "simulator.cpp"
)
