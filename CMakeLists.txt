cmake_minimum_required(VERSION 3.10)
project(emd-gaf C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 库类型: ON=动态库(.so)  OFF=静态库(.a)
option(BUILD_AS_SHARED "Build libemd_gaf as shared library" ON)
option(BUILD_EXAMPLES "Build examples" ON)

# ── Toolchain (set via -DCMAKE_TOOLCHAIN_FILE for cross-compile) ──
if(DEFINED CMAKE_TOOLCHAIN_FILE)
    message(STATUS "Cross-compiling with ${CMAKE_TOOLCHAIN_FILE}")
endif()

# ── Source directories ──
set(DRIVER_DIR  ${CMAKE_SOURCE_DIR}/drivers/icm45608/imu)
set(ICT1531_DIR ${CMAKE_SOURCE_DIR}/drivers/Ict1531x)
set(HAL_DIR     ${CMAKE_SOURCE_DIR}/hal)
set(APP_DIR     ${CMAKE_SOURCE_DIR}/app)
set(TOOLS_DIR   ${CMAKE_SOURCE_DIR}/tools)
set(SRC_DIR     ${CMAKE_SOURCE_DIR}/src)
set(INC_DIR     ${CMAKE_SOURCE_DIR}/inc)
set(EXAMPLE_DIR ${CMAKE_SOURCE_DIR}/example)

# ── Dependencies ──
set(GPIOD_DIR ${HAL_DIR}/gpiod)
# ── libgpiod: find_library for native + cross, fallback bundled
find_library(GPIOD_SYS gpiod)
if(GPIOD_SYS)
    set(GPIOD_LIB ${GPIOD_SYS})
    message(STATUS "gpiod: ${GPIOD_LIB}")
elseif(CMAKE_CROSSCOMPILING)
    set(GPIOD_LIB ${GPIOD_DIR}/lib/libgpiod.so)
    message(STATUS "gpiod (cross): bundled ${GPIOD_LIB}")
else()
    message(WARNING "No system libgpiod — linking will fail. Install libgpiod-dev.")
    set(GPIOD_LIB gpiod)  # let linker fail with clear message
endif()
find_library(PTHREAD_LIB pthread REQUIRED)
find_library(M_LIB m REQUIRED)

# ── Include paths ──
set(INC_DIRS
    ${GPIOD_DIR}/include
    ${DRIVER_DIR}/../..
    ${DRIVER_DIR}/..
    ${ICT1531_DIR}/..
    ${HAL_DIR}
    ${APP_DIR}
    ${TOOLS_DIR}
    ${TOOLS_DIR}/Invn
    ${TOOLS_DIR}/Invn/EmbUtils
    ${INC_DIR}
)

# ═══════════════════════════════════════════════════════════════════
# 公共源文件 (驱动层 + HAL + 磁力计适配)
# ═══════════════════════════════════════════════════════════════════

# IMU driver sources
set(IMU_SRCS
    ${DRIVER_DIR}/inv_imu_driver.c
    ${DRIVER_DIR}/inv_imu_driver_advanced.c
    ${DRIVER_DIR}/inv_imu_edmp.c
    ${DRIVER_DIR}/inv_imu_edmp_wearable.c
    ${DRIVER_DIR}/inv_imu_edmp_mrm.c
    ${DRIVER_DIR}/inv_imu_transport.c
    ${DRIVER_DIR}/inv_imu_i2cm.c
    ${DRIVER_DIR}/inv_imu_selftest.c
    ${DRIVER_DIR}/sif_classifier_ir.c
    ${DRIVER_DIR}/sif_feature_extract_ir.c
)

# Magnetometer driver sources
set(MAG_SRCS
    ${ICT1531_DIR}/Ict1531x.c
)

# Tools
set(TOOLS_SRCS
    ${TOOLS_DIR}/Invn/EmbUtils/Message.c
)

# HAL
set(HAL_SRCS
    ${HAL_DIR}/emd_hal.c
)

# Mag adapter (needed by both lib and app)
set(MAG_ADAPTER_SRCS
    ${APP_DIR}/invn_mag_ict1531x.c
)

# ═══════════════════════════════════════════════════════════════════
# 目标 1: libemd_gaf.{so,a} — 动态/静态库
# ═══════════════════════════════════════════════════════════════════

set(LIB_SRCS
    ${SRC_DIR}/emd_gaf.c
)

if(BUILD_AS_SHARED)
    add_library(emd_gaf SHARED
        ${IMU_SRCS}
        ${MAG_SRCS}
        ${MAG_ADAPTER_SRCS}
        ${TOOLS_SRCS}
        ${HAL_SRCS}
        ${LIB_SRCS}
    )
    message(STATUS "Building libemd_gaf as SHARED library (.so)")
else()
    add_library(emd_gaf STATIC
        ${IMU_SRCS}
        ${MAG_SRCS}
        ${MAG_ADAPTER_SRCS}
        ${TOOLS_SRCS}
        ${HAL_SRCS}
        ${LIB_SRCS}
    )
    message(STATUS "Building libemd_gaf as STATIC library (.a)")
endif()

target_include_directories(emd_gaf PUBLIC
    $<BUILD_INTERFACE:${INC_DIR}>
    $<INSTALL_INTERFACE:include>
    PRIVATE ${INC_DIRS}
)
target_link_libraries(emd_gaf PUBLIC ${GPIOD_LIB} ${PTHREAD_LIB} ${M_LIB})
target_compile_options(emd_gaf PRIVATE -Wall -Wextra -O2)

# 安装
install(TARGETS emd_gaf
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY inc/ DESTINATION include/emd_gaf)

# ═══════════════════════════════════════════════════════════════════
# 目标 2: emd-gaf — 独立可执行文件 (保留 app/ 原有功能)
# ═══════════════════════════════════════════════════════════════════

# App sources (原有可执行文件)
set(APP_SRCS
    ${APP_DIR}/linux_main.c
)

add_executable(emd-gaf
    ${IMU_SRCS}
    ${MAG_SRCS}
    ${MAG_ADAPTER_SRCS}
    ${APP_SRCS}
    ${TOOLS_SRCS}
    ${HAL_SRCS}
)

target_include_directories(emd-gaf PRIVATE ${INC_DIRS})
target_link_libraries(emd-gaf PRIVATE ${GPIOD_LIB} ${PTHREAD_LIB} ${M_LIB})
target_compile_options(emd-gaf PRIVATE -Wall -Wextra -O2)

install(TARGETS emd-gaf DESTINATION bin)

# ═══════════════════════════════════════════════════════════════════
# 目标 3: read_sensor — 示例程序 (链接 libemd_gaf)
# ═══════════════════════════════════════════════════════════════════

if(BUILD_EXAMPLES)
    add_executable(read_sensor ${EXAMPLE_DIR}/read_sensor.c)
    target_link_libraries(read_sensor emd_gaf)
    install(TARGETS read_sensor DESTINATION bin)
endif()
