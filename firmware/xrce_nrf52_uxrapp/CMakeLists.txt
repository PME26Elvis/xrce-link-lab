cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xrce_nrf52_uxrapp)

# 先放入本 app 的最小檔案
target_sources(app PRIVATE
  src/main.c
  src/transport_zephyr_serial.c
)

# 可選：micro-xrce-dds-client 原始碼（只有在存在時才編）
set(UXRCLIENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/uxrclient CACHE PATH "Path to micro-xrce-dds-client")

if (EXISTS ${UXRCLIENT_DIR}/include/uxr/client/client.h)
  message(STATUS "Building with micro-xrce-dds-client from: ${UXRCLIENT_DIR}")
  target_compile_definitions(app PRIVATE USE_UXRCLIENT=1)

  target_include_directories(app PRIVATE
    ${UXRCLIENT_DIR}/include
  )

  target_sources(app PRIVATE
    ${UXRCLIENT_DIR}/src/c/core/session/session.c
    ${UXRCLIENT_DIR}/src/c/core/session/session_info.c
    ${UXRCLIENT_DIR}/src/c/core/serialization/xrce_header.c
    ${UXRCLIENT_DIR}/src/c/core/serialization/xrce_protocol.c
    ${UXRCLIENT_DIR}/src/c/core/serialization/xrce_subheader.c
    ${UXRCLIENT_DIR}/src/c/core/transport/serial_protocol.c
    ${UXRCLIENT_DIR}/src/c/util/time.c
  )
else()
  message(STATUS "micro-xrce-dds-client not found; building heartbeat-only app.")
endif()
