cmake_minimum_required(VERSION 3.5.0)

# 交叉编译工具链
include(cross.cmake)

# 基础路径与项目名称配置
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)  
get_filename_component(PROJ_NAME ${PARENT_DIR} NAME)           # 取上级目录名作为项目名
project(${PROJ_NAME} VERSION 0.1.0 LANGUAGES C CXX)

# 编译参数（龙芯LoongArch适配）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(ENABLE_TERMINAL_DEBUG "Enable terminal crash debug symbols/flags" ON)

# 通用编译优化与龙芯架构参数
set(COMMON_FLAGS "-O3 -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

if (ENABLE_TERMINAL_DEBUG)
    message(STATUS "ENABLE_TERMINAL_DEBUG=ON -> add -g -O0 -fno-omit-frame-pointer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fno-omit-frame-pointer")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fno-omit-frame-pointer")
    add_compile_definitions(ENABLE_TERMINAL_DEBUG=1)
endif()

# OpenCV依赖配置
set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build") 
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_INCLUDE: ${OpenCV_INCLUDE_DIRS}, OpenCV_LIBS: ${OpenCV_LIBS}")

# 头文件路径汇总（拆分TFLM路径）
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    # 业务代码头文件
    ../user 
    ../code 
    ../model
    ../../libraries/zf_common 
    ../../libraries/zf_device 
    ../../libraries/zf_driver
    ../../libraries/zf_components/seekfree_assistant
    ../../libraries/zf_components/vofa_plus/src
    
    # TFLM核心头文件（完全拆分）
    ../../libraries/zf_components/tflm
    ../../libraries/zf_components/tflm/third_party/flatbuffers/include
    ../../libraries/zf_components/tflm/third_party/gemmlowp
    ../../libraries/zf_components/tflm/third_party/gemmlowp/fixedpoint
    ../../libraries/zf_components/tflm/third_party/ruy
    ../../libraries/zf_components/tflm/third_party/ruy/ruy
    ../../libraries/zf_components/tflm/tensorflow/lite/micro
    ../../libraries/zf_components/tflm/tensorflow/lite/micro/kernels
    ../../libraries/zf_components/tflm/tensorflow/lite/kernels/internal
    ../../libraries/zf_components/tflm/tensorflow/lite/core/api
    ../../libraries/zf_components/tflm/tensorflow/lite/core/c
    ../../libraries/zf_components/tflm/tensorflow/lite/schema
    "/home/lq/LS2K0300_Library/LS2K300_Library/Seekfree_LS2K0300_Opensource_Library/libraries/zf_components/ncnn/include"
)

# 静态库路径
link_directories(
    ../../libraries/zf_components/tflm/lib
    ../../libraries/zf_components/ncnn/lib

)

# 业务源文件
aux_source_directory(../user DIR_SRCS)
aux_source_directory(../code DIR_SRCS)
aux_source_directory(../model DIR_SRCS)
aux_source_directory(../../libraries/zf_common DIR_SRCS)
aux_source_directory(../../libraries/zf_device DIR_SRCS)
aux_source_directory(../../libraries/zf_driver DIR_SRCS)
aux_source_directory(../../libraries/zf_components/seekfree_assistant DIR_SRCS)
option(ENABLE_VOFA "Enable VOFA+ debug (JustFloat)" ON)
option(VOFA_USE_TCP "VOFA+ over TCP client (else USB serial)" ON)
set(VOFA_TCP_IP "192.168.137.1" CACHE STRING "VOFA+ PC IP (TCP server)")
set(VOFA_TCP_PORT 1347 CACHE STRING "VOFA+ TCP server port")

set(VOFA_PLUS_SRC "")
if (ENABLE_VOFA)
    list(APPEND VOFA_PLUS_SRC
        ../../libraries/zf_components/vofa_plus/src/vofa_client.c)
    if (VOFA_USE_TCP)
        list(APPEND VOFA_PLUS_SRC
            ../../libraries/zf_components/vofa_plus/src/vofa_port_tcp.cpp)
    else()
        list(APPEND VOFA_PLUS_SRC
            ../../libraries/zf_components/vofa_plus/src/vofa_port_linux.c)
    endif()
endif()

# 生成可执行文件 & 配置属性
add_executable(${PROJ_NAME} ${DIR_SRCS} ${VOFA_PLUS_SRC})

option(TRACK_DEBUG_LEVEL "Track debug: 0=off 1=HUD 2=HUD+serial" 0)
target_compile_definitions(${PROJ_NAME} PRIVATE TRACK_DEBUG_LEVEL=${TRACK_DEBUG_LEVEL})
if (ENABLE_VOFA)
    target_compile_definitions(${PROJ_NAME} PRIVATE ENABLE_VOFA=1)
    message(STATUS "ENABLE_VOFA=ON")
    if (VOFA_USE_TCP)
        target_compile_definitions(${PROJ_NAME} PRIVATE
            VOFA_USE_TCP=1
            VOFA_TCP_IP="${VOFA_TCP_IP}"
            VOFA_TCP_PORT=${VOFA_TCP_PORT})
        message(STATUS "VOFA transport: TCP ${VOFA_TCP_IP}:${VOFA_TCP_PORT}")
    else()
        message(STATUS "VOFA transport: serial")
    endif()
else()
    message(STATUS "ENABLE_VOFA=OFF")
endif()
if (DEFINED VOFA_SERIAL_DEVICE)
    target_compile_definitions(${PROJ_NAME} PRIVATE VOFA_SERIAL_DEVICE="${VOFA_SERIAL_DEVICE}")
endif()

set_target_properties(${PROJ_NAME} PROPERTIES
    INSTALL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib"
    BUILD_WITH_INSTALL_RPATH ON
)

# 链接依赖库
target_link_libraries(
    ${PROJ_NAME} 
    ${OpenCV_LIBS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../liblq_camera_ex.a
    tflm          
    ncnn          # <--- 新增：链接 NCNN 动态库
    gomp          # <--- 新增：链接 OpenMP 库 (我们刚传进板子的那个)
    pthread       
    m             
    dl            
)