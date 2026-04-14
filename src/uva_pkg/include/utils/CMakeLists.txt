cmake_minimum_required(VERSION 3.15)

project(rknn_model_zoo_utils)

add_library(fileutils STATIC
    file_utils.c
)
target_include_directories(fileutils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(imagedrawing STATIC
    image_drawing.c
)
target_include_directories(imagedrawing PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# add_definitions(-DLIBRGA_IM2D_HANDLE)
# rga
set(RGA_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../rga/lib/librga.so)
set(RGA_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../rga/include)
include_directories(${RGA_INCLUDES})

# stb
set(STB_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../stb)
include_directories(${STB_INCLUDES})

# jpeg_turbo
set(JPEG_TURBO_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../jpeg_turbo/lib/libturbojpeg.a)
set(JPEG_TURBO_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../jpeg_turbo/include)
include_directories(${JPEG_TURBO_INCLUDES})




add_library(imageutils STATIC
    image_utils.c
)
target_include_directories(imageutils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(imageutils
    ${JPEG_TURBO_LIB}
    ${RGA_LIB}
)

target_include_directories(imageutils PUBLIC
    ${STB_INCLUDES}
    ${JPEG_TURBO_INCLUDES}
    ${RGA_INCLUDES}
)
