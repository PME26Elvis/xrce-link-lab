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
    ${cmd}=    Set Variable    sed "s|__ELF_PATH__|${ELF}|g" ${RESC} > ${RESC_TMP}
    Run Process    bash    -lc    ${cmd}    shell=True
    File Should Exist    ${RESC_TMP}
    ${p}=    Start Process    bash    -lc    renode -e "s @${RESC_TMP}; start; sleep 3; q" > ${RENODE_LOG} 2>&1    shell=True
    Wait For Process    ${p}    timeout=40s
    ${pty}=    Run Process    bash    -lc    "grep -Eo '/dev/pts/[0-9]+' ${RENODE_LOG} | tail -n1"    shell=True    stdout=PTY
    RETURN    ${PTY}

*** Test Cases ***
UART heartbeat can be captured from PTY
    [Documentation]    從 Renode 取得 PTY，讀取 2 秒輸出，應包含 XRCE-STUB heartbeat。
    File Should Exist    ${ELF}
    ${PTY}=    Start Renode And Get PTY
    Run Keyword If    '${PTY}' == ''    Fail    Could not determine PTY path from Renode log.

    ${reader}=    Start Process    bash    -lc    "timeout 2s cat ${PTY} > ${UART_LOG}"    shell=True
    Wait For Process    ${reader}    timeout=10s

    File Should Exist    ${UART_LOG}
    ${sz}=    Get File Size    ${UART_LOG}
    Should Be True    ${sz} > 0

    ${hit}=    Run Process    bash    -lc    "grep -c 'XRCE-STUB heartbeat' ${UART_LOG} || true"    shell=True    stdout=COUNT
    Should Not Be Equal    ${COUNT}    0
