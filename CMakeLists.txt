cmake_minimum_required(VERSION 3.10)
project(emd-gaf C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ── Toolchain (set via -DCMAKE_TOOLCHAIN_FILE for cross-compile) ──
if(DEFINED CMAKE_TOOLCHAIN_FILE)
    message(STATUS "Cross-compiling with ${CMAKE_TOOLCHAIN_FILE}")
endif()

# ═══════════════════════════════════════════════════════════════════
# 目录定义 (重组后)
# ═══════════════════════════════════════════════════════════════════

set(PROJECT_ROOT   ${CMAKE_SOURCE_DIR})
set(INC_DIR        ${PROJECT_ROOT}/inc)
set(SRC_DIR        ${PROJECT_ROOT}/src)
set(DRIVER_DIR     ${SRC_DIR}/driver)
set(IMU_DIR        ${DRIVER_DIR}/icm45608/imu)
set(ICT1531_DIR    ${DRIVER_DIR}/Ict1531x)
set(HAL_DIR        ${SRC_DIR}/hal)
set(TOOLS_DIR      ${SRC_DIR}/tools)
set(DEMO_DIR       ${PROJECT_ROOT}/demo)
set(EXAMPLE_DIR    ${PROJECT_ROOT}/example)

# ── Dependencies ──
set(GPIOD_DIR ${HAL_DIR}/gpiod)

find_library(GPIOD_SYS gpiod)
if(GPIOD_SYS)
    set(GPIOD_LIB ${GPIOD_SYS})
    message(STATUS "gpiod: ${GPIOD_LIB}")
elseif(EXISTS ${GPIOD_DIR}/lib/libgpiod.so)
    set(GPIOD_LIB ${GPIOD_DIR}/lib/libgpiod.so)
    if(CMAKE_CROSSCOMPILING)
        message(STATUS "gpiod (cross): bundled ${GPIOD_LIB}")
    else()
        message(STATUS "gpiod (native): bundled ${GPIOD_LIB}")
    endif()
else()
    message(WARNING "No system or bundled libgpiod — linking will fail. Install libgpiod-dev.")
    set(GPIOD_LIB gpiod)
endif()
find_library(PTHREAD_LIB pthread REQUIRED)
find_library(M_LIB m REQUIRED)

# ── Include paths ──
set(INC_DIRS
    ${GPIOD_DIR}/include          # gpiod.h
    ${DRIVER_DIR}/icm45608        # imu/inv_imu_driver.h (driver includes use "imu/xxx.h")
    ${DRIVER_DIR}                 # Ict1531x/Ict1531x.h (mag driver includes use "Ict1531x/xxx.h")
    ${HAL_DIR}                    # system_interface.h, emd_hal.h
    ${SRC_DIR}                    # frontend.h, invn_mag.h
    ${TOOLS_DIR}                  # InvError.h
    ${TOOLS_DIR}/Invn/EmbUtils    # Message.h
    ${INC_DIR}                    # emd_gaf.h, emd_gaf_types.h
)

# ═══════════════════════════════════════════════════════════════════
# 源文件
# ═══════════════════════════════════════════════════════════════════

# IMU driver
set(IMU_SRCS
    ${IMU_DIR}/inv_imu_driver.c
    ${IMU_DIR}/inv_imu_driver_advanced.c
    ${IMU_DIR}/inv_imu_edmp.c
    ${IMU_DIR}/inv_imu_edmp_wearable.c
    ${IMU_DIR}/inv_imu_edmp_mrm.c
    ${IMU_DIR}/inv_imu_transport.c
    ${IMU_DIR}/inv_imu_i2cm.c
    ${IMU_DIR}/inv_imu_selftest.c
    ${IMU_DIR}/sif_classifier_ir.c
    ${IMU_DIR}/sif_feature_extract_ir.c
)

# Magnetometer driver
set(MAG_SRCS
    ${ICT1531_DIR}/Ict1531x.c
)

# Mag adapter (needed by lib)
set(MAG_ADAPTER_SRCS
    ${SRC_DIR}/invn_mag_ict1531x.c
)

# Tools
set(TOOLS_SRCS
    ${TOOLS_DIR}/Invn/EmbUtils/Message.c
)

# HAL
set(HAL_SRCS
    ${HAL_DIR}/emd_hal.c
)

# Library core
set(LIB_CORE_SRCS
    ${SRC_DIR}/emd_gaf.c
)

# ═══════════════════════════════════════════════════════════════════
# 目标 1: libemd_gaf.so — 动态库
# ═══════════════════════════════════════════════════════════════════

add_library(emd_gaf SHARED
    ${IMU_SRCS}
    ${MAG_SRCS}
    ${MAG_ADAPTER_SRCS}
    ${TOOLS_SRCS}
    ${HAL_SRCS}
    ${LIB_CORE_SRCS}
)

target_include_directories(emd_gaf
    PUBLIC
        $<BUILD_INTERFACE:${INC_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${INC_DIRS}
)

target_link_libraries(emd_gaf
    PUBLIC ${GPIOD_LIB} ${PTHREAD_LIB} ${M_LIB}
)

target_compile_options(emd_gaf PRIVATE -Wall -Wextra -O2)

# 安装
install(TARGETS emd_gaf
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY inc/ DESTINATION include/emd_gaf)

# ═══════════════════════════════════════════════════════════════════
# 目标 2: emd-gaf — 独立可执行 (链接 libemd_gaf.so)
# ═══════════════════════════════════════════════════════════════════

add_executable(emd-gaf ${DEMO_DIR}/emd-gaf.c)
target_link_libraries(emd-gaf emd_gaf)
target_compile_options(emd-gaf PRIVATE -Wall -Wextra -O2)

install(TARGETS emd-gaf DESTINATION bin)

# ═══════════════════════════════════════════════════════════════════
# 目标 3: read_sensor — 示例程序 (链接 libemd_gaf.so)
# ═══════════════════════════════════════════════════════════════════

add_executable(read_sensor ${EXAMPLE_DIR}/read_sensor.c)
target_link_libraries(read_sensor emd_gaf)
target_compile_options(read_sensor PRIVATE -Wall -Wextra -O2)

install(TARGETS read_sensor DESTINATION bin)
