*** Settings ***
Library           Process
Library           OperatingSystem
Library           Remote    127.0.0.1:9999    # 連接到 Renode 的 Robot Server
Library           String
*** Variables ***
${ELF}            build/zephyr/zephyr.elf
${RESC}           ${CURDIR}${/}..${/}renode${/}nrf52_uart_capture.resc
${RESC_TMP}       ${CURDIR}${/}..${/}renode${/}_nrf52_uart_capture.tmp.resc
${RENODE_LOG}     ${CURDIR}${/}..${/}uart_capture.renode.log
${UART_LOG}       ${CURDIR}${/}..${/}uart_capture.device.log

*** Keywords ***
Start Renode And Get PTY
    ${cmd}=    Set Variable  sed "s|__ELF_PATH__|${ELF}|g" ${RESC} > ${RESC_TMP}
    Run Process    bash  -lc    ${cmd}    shell=True
    File Should Exist    ${RESC_TMP}

    # 【最終方案 v7】使用 --disable-xwt 模式啟動，並由 Robot 遠端載入腳本，確保 Renode 不會提前退出
    ${p}=    Start Process    renode    --disable-xwt    --robot-server-port    9999    stdout=${RENODE_LOG}    stderr=STDOUT
    Sleep    5s    # 在 CI 環境中，給予更長的啟動時間以確保 Robot Server 完全就緒

    # 透過 Remote Library 執行 Renode 關鍵字
    # 遠端載入腳本
    Run Keyword    Execute Command    s @${RESC_TMP}
    # 啟動模擬
    Run Keyword    Start Emulation

    # 查詢 PTY 屬性
    ${pty}=    Run Keyword    Get Machine Uart PTY    nrf52-ci-capture    sysbus.uart0

    # 關閉 Renode 並清理 (透過 Remote Library)
    Run Keyword    Stop Emulation
    Run Keyword    Shutdown Renode

    Wait For Process    ${p}    timeout=20s
    RETURN    ${pty}

*** Test Cases ***
UART heartbeat can be captured from PTY
    [Documentation]    從 Renode 取得 PTY，讀取 2 秒輸出，應包含 XRCE-STUB heartbeat。
    File Should Exist    ${ELF}
    ${PTY}=    Start Renode And Get PTY
    Run Keyword If    '${PTY}' == ''    Fail    Could not determine PTY path from Renode log. Check ${RENODE_LOG}.

    # 讀取 PTY 2 秒並寫入檔案（用 cat 搭配 timeout）
    ${reader}=    Start Process    bash  -lc    "timeout 2s cat ${PTY} > ${UART_LOG}"    shell=True
    Wait For Process    ${reader}    timeout=10s

    File Should Exist    ${UART_LOG}
    ${sz}=    Get File Size    ${UART_LOG}
    Should Be True    ${sz} > 0

    # 你的 stub app 會每 500ms 印出 XRCE-STUB heartbeat
    ${output}=    Get File    ${UART_LOG}
    Should Contain    ${output}    XRCE-STUB heartbeat

*** Keywords ***
Get Machine Uart PTY
    [Arguments]    ${machine_name}    ${uart_name}
    ${output}=    Execute Command    ${machine_name} ${uart_name}
    ${matches}=    Get Regexp Matches    ${output}    Host file: (.*)
    RETURN    ${matches[0]}

Shutdown Renode
    Execute Command    q
