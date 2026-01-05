cmake_minimum_required(VERSION 3.20.0)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/driver/zephyr-dw3000-decadriver/)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(05012026_Read_CIR_SS_TWR_Initiator)

target_sources(app PRIVATE src/main.c)
zephyr_library_compile_options(-O1) # Needed by the DW3000 driver
target_compile_options(app PRIVATE)
target_sources(app PRIVATE src/main.c)

target_sources(app PRIVATE driver/shared_data/shared_data.c)
target_include_directories(app PRIVATE driver/shared_data/.)
