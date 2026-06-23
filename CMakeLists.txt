cmake_minimum_required(VERSION 3.10)
project(emd-gaf C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ── Toolchain (set via -DCMAKE_TOOLCHAIN_FILE for cross-compile) ──
if(DEFINED CMAKE_TOOLCHAIN_FILE)
    message(STATUS "Cross-compiling with ${CMAKE_TOOLCHAIN_FILE}")
endif()

# ── Dependencies (only pthread needed) ──
find_library(PTHREAD_LIB pthread REQUIRED)
find_library(M_LIB m REQUIRED)

# ── Sources ──
set(DRIVER_DIR  ${CMAKE_SOURCE_DIR}/drivers/icm45608/imu)
set(ICT1531_DIR ${CMAKE_SOURCE_DIR}/drivers/Ict1531x)
set(HAL_DIR     ${CMAKE_SOURCE_DIR}/hal)
set(APP_DIR     ${CMAKE_SOURCE_DIR}/app)
set(TOOLS_DIR   ${CMAKE_SOURCE_DIR}/tools)

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

# App sources
set(APP_SRCS
    ${APP_DIR}/linux_main.c
    ${APP_DIR}/invn_mag_ict1531x.c
)

# Tools
set(TOOLS_SRCS
    ${TOOLS_DIR}/Invn/EmbUtils/Message.c
)

# HAL
set(HAL_SRCS
    ${HAL_DIR}/emd_hal.c
)

# ── Include paths ──
set(INC_DIRS
    ${DRIVER_DIR}/../..
    ${DRIVER_DIR}/..
    ${ICT1531_DIR}/..
    ${HAL_DIR}
    ${APP_DIR}
    ${TOOLS_DIR}
    ${TOOLS_DIR}/Invn
    ${TOOLS_DIR}/Invn/EmbUtils
)

# ── Target ──
add_executable(emd-gaf
    ${IMU_SRCS}
    ${MAG_SRCS}
    ${APP_SRCS}
    ${TOOLS_SRCS}
    ${HAL_SRCS}
)

target_include_directories(emd-gaf PRIVATE ${INC_DIRS})
target_link_libraries(emd-gaf PRIVATE ${PTHREAD_LIB} ${M_LIB})
target_compile_options(emd-gaf PRIVATE -Wall -Wextra -O2)

# ── Install ──
install(TARGETS emd-gaf DESTINATION bin)
