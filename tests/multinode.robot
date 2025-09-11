*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}

*** Variables ***
# 讓 UART 等待別超時；官方建議可自訂此變數
${DEFAULT_UART_TIMEOUT}    20

*** Test Cases ***
BLE Two-Node Demo Boots Both Zephyr Nodes
    # 載入我們的 helper .resc（內含官方兩節點 BLE demo）
    Execute Command    include @${CURDIR}/../renode/multinode_ble.resc

    # 對 central 的 UART0 建立 tester
    Execute Command    mach set "central"
    ${central_uart}=   Create Terminal Tester    sysbus.uart0    timeout=${DEFAULT_UART_TIMEOUT}    defaultPauseEmulation=true

    # 對 peripheral 的 UART0 建立 tester
    Execute Command    mach set "peripheral"
    ${peripheral_uart}=    Create Terminal Tester    sysbus.uart0    timeout=${DEFAULT_UART_TIMEOUT}    defaultPauseEmulation=true

    # 啟動兩機模擬
    Start Emulation

    # 兩台都應該印出 Zephyr 開機行（大量官方範例與儀表板都用這行作 smoke 檢查）
    Wait For Line On Uart    Booting Zephyr OS    testerId=${central_uart}
    Wait For Line On Uart    Booting Zephyr OS    testerId=${peripheral_uart}
