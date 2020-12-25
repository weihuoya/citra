add_library(lzo STATIC minilzo.c)
target_include_directories(lzo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
