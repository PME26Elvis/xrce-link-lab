*** Settings ***
Resource        ${RENODEKEYWORDS}
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown

*** Variables ***
${RESC}         @${CURDIR}/../renode/two_nodes_nrf.resc
${PROMPT}       uart:~$
${TIMEOUT}      90

*** Test Cases ***
NRF Two-Node Zephyr Shell Prompts Appear
    [Documentation]    確認兩台模擬的 nRF 節點都能跑起 Zephyr shell，並各自出現提示字元。
    Execute Command    include ${RESC}

    ${uartA}=          Create Terminal Tester    sysbus.uart0    machine=nodeA    timeout=${TIMEOUT}    defaultPauseEmulation=true
    ${uartB}=          Create Terminal Tester    sysbus.uart0    machine=nodeB    timeout=${TIMEOUT}    defaultPauseEmulation=true

    Start Emulation

    Wait For Prompt On Uart    ${PROMPT}    testerId=${uartA}
    Wait For Prompt On Uart    ${PROMPT}    testerId=${uartB}
