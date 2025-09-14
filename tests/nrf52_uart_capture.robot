*** Settings ***
Library           Process
Library           OperatingSystem
Library           Telnet
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

    # 【最終方案 v4】啟動 Renode 並開放 Monitor TCP port，用 Telnet 直接查詢 PTY 路徑
    ${p}=    Start Process    renode    --port    1234    -e    s @${RESC_TMP}; start    stdout=${RENODE_LOG}    stderr=STDOUT
    Sleep    2s    # 等待 Renode 完全啟動

    # 連接到 Renode Monitor
    Open Connection    127.0.0.1    port=1234    timeout=10s
    Login    ${EMPTY}    ${EMPTY}    # Renode Monitor 不需要帳號密碼

    # 切換到正確的機器上下文並查詢 PTY 屬性
    Write    mach set "nrf52-ci-capture"
    ${output}=    Read Until Prompt    (nrf52-ci-capture)
    Write    get-property sysbus.uart0.FileName
    ${pty_output}=    Read Until Prompt    (nrf52-ci-capture)

    # 從回傳結果中提取 PTY 路徑
    ${matches}=    Get Regexp Matches    ${pty_output}    (/dev/pts/\\d+)
    ${pty}=    Set Variable If    len(${matches}) > 0    ${matches[0]}    ${EMPTY}

    # 關閉 Renode 並清理
    Write    q
    Close Connection
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
