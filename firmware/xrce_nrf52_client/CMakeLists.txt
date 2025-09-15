cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xrce_nrf52_client)

# uxrclient 最小子集 (只要 core + serial transport)
target_sources(app PRIVATE
  src/main.c
  src/transport_zephyr_serial.c
  third_party/uxrclient/src/c/core/session/session.c
  third_party/uxrclient/src/c/core/session/session_info.c
  third_party/uxrclient/src/c/core/serialization/xrce_header.c
  third_party/uxrclient/src/c/core/serialization/xrce_subheader.c
  third_party/uxrclient/src/c/core/serialization/xrce_protocol.c
  third_party/uxrclient/src/c/core/transport/serial_protocol.c
  third_party/uxrclient/src/c/util/time.c
)

target_include_directories(app PRIVATE
  third_party/uxrclient/include
)