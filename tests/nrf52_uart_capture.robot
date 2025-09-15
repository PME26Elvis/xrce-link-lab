*** Settings ***
Library           Process
Library           OperatingSystem
Library           String
Suite Setup       Remove File    ${UART_LOG}    # 每次先清空舊檔（忽略失敗）

*** Variables ***
${ELF}            build/zephyr/zephyr.elf
${RESC}           ${CURDIR}${/}..${/}renode${/}nrf52_uart_capture.resc
${RESC_TMP}       ${CURDIR}${/}..${/}renode${/}_nrf52_uart_capture.tmp.resc
${RENODE_LOG}     ${CURDIR}${/}..${/}uart_capture.renode.log
${UART_LOG}       ${CURDIR}${/}..${/}uart_capture.device.log
${PTY}            /tmp/xrce_uart

*** Keywords ***
Start Renode With Fixed PTY
    [Documentation]    用 sed 注入 ELF 路徑，跑 Renode（背景），等待固定 PTY 出現
    ${cmd}=    Set Variable    sed "s|__ELF_PATH__|${ELF}|g" "${RESC}" > "${RESC_TMP}"
    Run Process    bash    -lc    ${cmd}    shell=True
    File Should Exist       ${RESC_TMP}

    # 背景啟動 Renode（無 GUI），把輸出導到 log
    ${p}=    Start Process    bash    -lc    "renode --disable-xwt -e \"s @${RESC_TMP}; q\""    shell=True    stdout=${RENODE_LOG}    stderr=STDOUT

    # 等待 Renode 啟動並創建 PTY（最多重試 20 次，每次 0.2s）
    Wait Until Keyword Succeeds    20x    0.2s    File Should Exist    ${PTY}
    [Return]    ${p}

Capture From PTY For    [Arguments]    ${secs}
    [Documentation]    從固定 PTY 抓取輸出 ${secs} 秒
    ${reader}=    Start Process    bash    -lc    "timeout ${secs}s cat ${PTY} > ${UART_LOG}"    shell=True
    Wait For Process    ${reader}    timeout=${secs}s

*** Test Cases ***
UART heartbeat can be captured from PTY
    [Documentation]    從 /tmp/xrce_uart 讀取 2 秒輸出，應包含 XRCE-STUB heartbeat。
    File Should Exist    ${ELF}

    ${renode}=    Start Renode With Fixed PTY
    Capture From PTY For    2

    # 結束 Renode（它本來就會 q；保守起見等待）
    Wait For Process         ${renode}    timeout=40s

    File Should Exist        ${UART_LOG}
    ${sz}=    Get File Size  ${UART_LOG}
    Should Be True           ${sz} > 0

    ${output}=    Get File   ${UART_LOG}
    Should Contain           ${output}    XRCE-STUB heartbeat
