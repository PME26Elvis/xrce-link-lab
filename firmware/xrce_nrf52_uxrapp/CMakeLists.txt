cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xrce_nrf52_uxrapp)

# 永遠編的最小 app
target_sources(app PRIVATE
  src/main.c
)

# uxrclient 目錄（CI 的 fetch 步驟會把 repo 放到這裡）
set(UXRCLIENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/uxrclient CACHE PATH "Path to micro-xrce-dds-client")

if (EXISTS ${UXRCLIENT_DIR}/include/uxr/client/client.h)
  message(STATUS "Building with micro-xrce-dds-client from: ${UXRCLIENT_DIR}")
  target_compile_definitions(app PRIVATE USE_UXRCLIENT=1)
  target_include_directories(app PRIVATE ${UXRCLIENT_DIR}/include)

  # 我們自己的 serial transport（只有在 USE_UXRCLIENT 時才編）
  target_sources(app PRIVATE
    src/transport_zephyr_serial.c
  )

  # 精確收集 uxrclient 必要 .c
  file(GLOB_RECURSE UXR_CORE_C   ${UXRCLIENT_DIR}/src/c/core/*.c)
  file(GLOB         UXR_UTIL_C   ${UXRCLIENT_DIR}/src/c/util/*.c)
  file(GLOB         UXR_SERIAL_C ${UXRCLIENT_DIR}/src/c/transport/serial/*.c)

  target_sources(app PRIVATE
    ${UXR_CORE_C}
    ${UXR_UTIL_C}
    ${UXR_SERIAL_C}
  )
else()
  message(STATUS "micro-xrce-dds-client NOT found; building heartbeat-only app.")
endif()
