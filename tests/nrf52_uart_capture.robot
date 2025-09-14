*** Settings ***
Library           Process
Library           OperatingSystem

*** Variables ***
${ELF}            build/zephyr/zephyr.elf
${RESC}           renode/nrf52_load.resc
${RESC_TMP}       renode/_nrf52_uart_capture.resc
${RENODE_LOG}     uart_capture.renode.log
${UART_LOG}       uart_capture.device.log

*** Keywords ***
Start Renode And Get PTY
    ${cmd}=    Set Variable  sed "s|__ELF_PATH__|${ELF}|g" ${RESC} > ${RESC_TMP}
    Run Process    bash  -lc    ${cmd}    shell=True
    File Should Exist    ${RESC_TMP}
    # 開 Renode，讓它跑 3 秒再退出；同時把 log 存起來
    ${p}=    Start Process    bash  -lc    "renode -e 's @${RESC_TMP}; start; sleep 3; q' > ${RENODE_LOG} 2>&1"    shell=True
    Wait For Process    ${p}    timeout=40s
    # 從 log 找出 "PTY device path: /dev/pts/N" 並取出路徑 (更穩健的 grep + sed)
    Run Process    bash  -lc    "grep -o 'PTY device path: /dev/pts/[0-9]+' ${RENODE_LOG} | sed 's/PTY device path: //g' | tail -n1 > ${CURDIR}/pty.txt"    shell=True
    ${pty}=    Get File    ${CURDIR}/pty.txt
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
