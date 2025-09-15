cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xrce_nrf52_uxrapp)

# 這顆 app 的原始碼
target_sources(app PRIVATE
  src/main.c
  src/transport_zephyr_serial.c
)

# 引用 micro-xrce-dds-client 的最小子集（core + serial）
# 注意：第三方原始碼會在 CI 裡以固定路徑 fetch 下來
target_sources(app PRIVATE
  third_party/uxrclient/src/c/core/session/session.c
  third_party/uxrclient/src/c/core/session/session_info.c
  third_party/uxrclient/src/c/core/serialization/xrce_header.c
  third_party/uxrclient/src/c/core/serialization/xrce_protocol.c
  third_party/uxrclient/src/c/core/serialization/xrce_subheader.c
  third_party/uxrclient/src/c/core/transport/serial_protocol.c
  third_party/uxrclient/src/c/util/time.c
)

target_include_directories(app PRIVATE
  third_party/uxrclient/include
)
