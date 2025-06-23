cmake_minimum_required(VERSION 3.15)
project(dijkstra)

# --- Настройки компиляции ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Исходные файлы ---
file(GLOB_RECURSE SOURCE_FILES 
  "src/*.cpp"
  "./main.cpp"
)

# --- Создаем исполняемый файл ---
add_executable(dijkstra ${SOURCE_FILES})

# --- Указываем include-директории ---
target_include_directories(dijkstra PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
