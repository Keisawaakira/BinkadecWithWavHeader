cmake_minimum_required(VERSION 3.0.0)
project(binkadec VERSION 0.1.0 LANGUAGES C CXX)

#C++20
set(CMAKE_CXX_STANDARD 20)
include(CTest)
enable_testing()

#Check if SDK/BinkAudio dir exists
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/SDK/BinkAudio")
    message(FATAL_ERROR "Due to possible legal reasons, you must copy the BinkAudio SDK from https://github.com/EpicGames/UnrealEngine/blob/5.0/Engine/Source/Runtime/BinkAudioDecoder/SDK/BinkAudio to SDK/BinkAudio")
endif()

#SDK/BinkAudio/Src/*
file(GLOB BINKAUDIO_SRC
    "SDK/BinkAudio/Src/*.c"
    "SDK/BinkAudio/Src/*.cpp"
)
add_library(BinkAudio STATIC ${BINKAUDIO_SRC})
target_include_directories(BinkAudio PUBLIC "SDK/BinkAudio/Include")

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
)
FetchContent_MakeAvailable(argparse)

add_executable(binkadec main.cpp)
target_link_libraries(binkadec 
BinkAudio
argparse
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
