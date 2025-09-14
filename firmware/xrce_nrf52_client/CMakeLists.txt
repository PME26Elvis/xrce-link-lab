cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xrce_nrf52_client)

target_sources(app PRIVATE
  src/main.c
  src/transport_stub.c
)

# 之後把 eProsima uxrclient 換成真實傳輸時，只要改這裡把第三方源加進來即可
# target_sources(app PRIVATE third_party/uxrclient/src/c/profile/transport/serial/serial_transport.c ...)
