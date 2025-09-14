*** Settings ***
Library           Process
Library           OperatingSystem
Library           String
*** Variables ***
${ELF}            build/zephyr/zephyr.elf
${RESC}           ${CURDIR}${/}..${/}renode${/}nrf52_load.resc
${RESC_TMP}       ${CURDIR}${/}..${/}renode${/}_nrf52_uart_capture.resc
${RENODE_LOG}     ${CURDIR}${/}..${/}uart_capture.renode.log
${UART_LOG}       ${CURDIR}${/}..${/}uart_capture.device.log

*** Keywords ***
Start Renode And Get PTY
    ${cmd}=    Set Variable  sed "s|__ELF_PATH__|${ELF}|g" ${RESC} > ${RESC_TMP}
    Run Process    bash  -lc    ${cmd}    shell=True
    File Should Exist    ${RESC_TMP}
    # 開 Renode，讓它跑 3 秒再退出；同時把 log 存起來
    ${p}=    Start Process    bash  -lc    "renode -e 's @${RESC_TMP}; start; sleep 3; q' > ${RENODE_LOG} 2>&1"    shell=True
    Wait For Process    ${p}    timeout=40s

    # 【新方法】直接讀取 Renode log 檔案，在 Robot 內部用字串處理，避免檔案 I/O 競爭問題
    File Should Exist    ${RENODE_LOG}
    ${log_content}=    Get File    ${RENODE_LOG}
    # 用正規表示式從 log 內容中直接提取 PTY 路徑
    ${matches}=    Get Regexp Matches    ${log_content}    PTY device path: (/dev/pts/\\d+)    1
    # Get Regexp Matches 會返回一個 list，我們取第一個元素
    ${pty}=    Set Variable If    len(${matches}) > 0    ${matches[0]}    ${EMPTY}
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
