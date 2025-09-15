*** Settings ***
Library           Process
Library           OperatingSystem
Library           String

*** Variables ***
${ELF}            build/zephyr/zephyr.elf
${RESC}           ${CURDIR}${/}..${/}renode${/}nrf52_uart_capture.resc
${RESC_TMP}       ${CURDIR}${/}..${/}renode${/}_nrf52_uart_capture.tmp.resc
${RENODE_LOG}     ${CURDIR}${/}..${/}uart_capture.renode.log
${UART_LOG}       ${CURDIR}${/}..${/}uart_capture.device.log
${TCP_HOST}       127.0.0.1
${TCP_PORT}       34567

*** Keywords ***
Start Renode
    ${cmd}=    Set Variable    sed "s|__ELF_PATH__|${ELF}|g" "${RESC}" > "${RESC_TMP}"
    Run Process    bash    -lc    ${cmd}    shell=True
    File Should Exist       ${RESC_TMP}
    ${p}=    Start Process  bash  -lc  "renode --disable-xwt -e \"s @${RESC_TMP}; q\""    shell=True    stdout=${RENODE_LOG}    stderr=STDOUT
    [Return]    ${p}

Wait For Tcp Port
    [Arguments]    ${host}    ${port}
    # 等待最多 30 次，每次 0.2s，直到 127.0.0.1:${port} 監聽起來
    Wait Until Keyword Succeeds    30x    0.2s    Run Process    bash    -lc    "ss -ltn | grep -q \":${port} \""    shell=True

*** Test Cases ***
UART heartbeat can be captured over TCP
    [Documentation]    將 UART 綁到 TCP:34567，使用 socat 抓 2 秒輸出，應含 XRCE-STUB heartbeat。
    File Should Exist    ${ELF}

    ${renode}=    Start Renode
    Wait For Tcp Port    ${TCP_HOST}    ${TCP_PORT}

    # 用 socat 連線並抓 2 秒資料
    ${reader}=    Start Process    bash    -lc    "timeout 2s socat - TCP:${TCP_HOST}:${TCP_PORT} > ${UART_LOG}"    shell=True
    Wait For Process    ${reader}    timeout=10s

    # 等 Renode 自行退出（resc 中最後有 q）
    Wait For Process    ${renode}    timeout=40s

    File Should Exist        ${UART_LOG}
    ${sz}=    Get File Size  ${UART_LOG}
    Should Be True           ${sz} > 0

    ${output}=    Get File   ${UART_LOG}
    Should Contain           ${output}    XRCE-STUB heartbeat
