*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}

*** Variables ***
${DEFAULT_UART_TIMEOUT}    90

*** Test Cases ***
NRF Two-Node Zephyr Shell Prompts Appear
    # 載入我們的兩節點腳本（不自動 start）
    Execute Command    include @${CURDIR}/../renode/two_nodes_nrf.resc

    # 建立 nodeA 的 UART tester
    Execute Command    mach set "nodeA"
    ${uart_a}=    Create Terminal Tester    sysbus.uart0    timeout=${DEFAULT_UART_TIMEOUT}    defaultPauseEmulation=true

    # 建立 nodeB 的 UART tester
    Execute Command    mach set "nodeB"
    ${uart_b}=    Create Terminal Tester    sysbus.uart0    timeout=${DEFAULT_UART_TIMEOUT}    defaultPauseEmulation=true

    # 啟動兩台機器
    Start Emulation

    # 兩台都應該出現 Zephyr Shell 提示字串（官方示例使用的 prompt）
    Wait For Prompt On Uart    uart:~$    testerId=${uart_a}
    Wait For Prompt On Uart    uart:~$    testerId=${uart_b}
