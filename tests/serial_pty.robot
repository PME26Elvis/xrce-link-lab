*** Settings ***
Resource        ${RENODEKEYWORDS}
Library         Process
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown

*** Variables ***
${RESC}         @${CURDIR}/../renode/serial_pty_nrf.resc
${PROMPT}       uart:~$
${TIMEOUT}      90
${PTY}          /tmp/xrce_uart
${ARTDIR}       ${CURDIR}/../artifacts/serial_pty

*** Test Cases ***
Serial PTY bridges to Agent and Shell is reachable
    [Documentation]    建立主機 PTY 並橋接到模擬 UART0，啟動 Micro XRCE-DDS Agent（serial）後，確認 Zephyr Shell 出現提示字元。
    Execute Command    include ${RESC}

    # 確保 artifacts 目錄存在（Renode 的 runner 會在 repo 工作目錄下執行）
    ${_}=    Run Process    mkdir    -p    ${ARTDIR}

    # 在同一測試中啟動 Agent，避免跨 step 背景行程消失的問題
    Start Process    MicroXRCEAgent    serial    -d    ${PTY}    -b    115200    --verbose    4
    ...    stdout=${ARTDIR}/agent_serial.log    stderr=${ARTDIR}/agent_serial.log    alias=agent
    # 若 Agent 未成功啟動，上面 Start Process 會失敗；也可視需要加「Process Should Be Running  agent」

    # 連到 nodeS 的 UART0，等待 Zephyr Shell 提示
    ${uart}=    Create Terminal Tester    sysbus.uart0    machine=nodeS    timeout=${TIMEOUT}    defaultPauseEmulation=true
    Start Emulation
    Wait For Prompt On Uart    ${PROMPT}    testerId=${uart}

    # 清理 Agent
    Terminate Process    agent
