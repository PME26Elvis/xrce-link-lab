*** Settings ***
Library           Process
Library           OperatingSystem

*** Variables ***
${ELF}            build/zephyr/zephyr.elf
${RESC}           renode/nrf52_load.resc
${RESC_TMP}       renode/_nrf52_uart_capture.resc
${RENODE_LOG}     artifacts/uart_capture/uart_capture.renode.log
${UART_LOG}       artifacts/uart_capture/uart_capture.device.log

*** Keywords ***
Start Renode In Background And Get PTY
    # 準備 artifacts 目錄與替換後的 resc
    Run Process    bash    -lc    "mkdir -p artifacts/uart_capture"    shell=True
    ${cmd}=    Set Variable    sed "s|__ELF_PATH__|${ELF}|g" ${RESC} > ${RESC_TMP}
    Run Process    bash    -lc    ${cmd}    shell=True
    File Should Exist    ${RESC_TMP}

    # 以背景模式啟 Renode，不要 q；讓它保持執行
    ${renode}=    Start Process    bash    -lc    "renode -e 's @${RESC_TMP}; start' > ${RENODE_LOG} 2>&1"    shell=True
    Sleep    1.0s

    # 從 log 擷取 PTY 路徑（多試幾次以等 log 刷出）
    ${pty}=    Set Variable    ${EMPTY}
    :FOR    ${i}    IN RANGE    1    6
    \    ${pty}=    Run Process    bash    -lc    "grep -Eo '/dev/pts/[0-9]+' ${RENODE_LOG} | tail -n1 || true"    shell=True    stdout=PTY
    \    Run Keyword If    '${PTY}' != ''    Exit For Loop
    \    Sleep    0.5s

    RETURN    ${PTY}    ${renode}

*** Test Cases ***
UART heartbeat can be captured from PTY
    [Documentation]    同步啟 Renode、並行讀取 PTY 2 秒，期望看到 'XRCE-STUB heartbeat'。
    File Should Exist    ${ELF}
    ${PTY}    ${RENODE}=    Start Renode In Background And Get PTY
    Run Keyword If    '${PTY}' == ''    Fail    Could not determine PTY path from Renode log.

    # 並行讀 PTY → 檔案（先確保目錄存在）
    Run Process    bash    -lc    "timeout 2s cat ${PTY} > ${UART_LOG}"    shell=True
    File Should Exist    ${UART_LOG}
    ${sz}=    Get File Size    ${UART_LOG}
    Should Be True    ${sz} > 0

    # 你的 stub app 每 500ms 印 heartbeat，2 秒內至少應看到 1 次
    ${hit}=    Run Process    bash    -lc    "grep -c 'XRCE-STUB heartbeat' ${UART_LOG} || true"    shell=True    stdout=COUNT
    Should Not Be Equal    ${COUNT}    0

    # 收尾：停掉 Renode
    Terminate Process    ${RENODE}
