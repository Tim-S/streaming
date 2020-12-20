cmake_minimum_required(VERSION 3.13)
project(RasPIZeroAdcMcp3202Dma C)

#set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

# lots of warnings and all warnings as errors
add_compile_options(-Wall -Wextra -pedantic)

add_executable(rpi_disp_dma rpi_disp_dma.c rpi_dma_utils.c)
add_executable(rpi_adc_stream rpi_adc_stream.c rpi_dma_utils.c)
add_executable(rpi_adc_stream_tcp rpi_adc_stream_tcp.c rpi_dma_utils.c)
