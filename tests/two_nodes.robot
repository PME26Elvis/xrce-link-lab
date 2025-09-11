*** Settings ***
Resource            ${RENODEKEYWORDS}
Suite Setup         Setup
Test Setup          Test Setup
Test Teardown       Test Teardown

*** Variables ***
${RESC}             @scripts/two-nodes/nrf52-dual-demo.resc
${PROMPT}           uart:~$

*** Test Cases ***
NRF Two-Node Zephyr Shell Prompts Appear
    [Documentation] 確認兩台模擬的 nRF 節點都能跑起 Zephyr shell，並各自出現提示字元。
    Execute Command                 include ${RESC}

    # 關鍵修正：在 Create Terminal Tester 時，明確指定 machine=nodeA / nodeB
    ${uartA}=                       Create Terminal Tester    sysbus.uart0    machine=nodeA    timeout=10    defaultPauseEmulation=true
    ${uartB}=                       Create Terminal Tester    sysbus.uart0    machine=nodeB    timeout=10    defaultPauseEmulation=true

    Start Emulation

    Wait For Prompt On Uart         ${PROMPT}    testerId=${uartA}
    Wait For Prompt On Uart         ${PROMPT}    testerId=${uartB}
