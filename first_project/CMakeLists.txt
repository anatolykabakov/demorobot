cmake_minimum_required(VERSION 2.8.8)
project(mylittleproject)

add_library(logic STATIC logic.c)

add_executable(mylittleprogram test.c)

target_link_libraries(mylittleprogram logic)