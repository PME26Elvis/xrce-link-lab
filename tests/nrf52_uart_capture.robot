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

*** Keywords ***
Start Renode And Extract PTY
    [Documentation]    用 sed 注入 ELF 路徑、啟 Renode、從 log 萃取 'PTY: /dev/pts/N'
    ${cmd}=    Set Variable    sed "s|__ELF_PATH__|${ELF}|g" "${RESC}" > "${RESC_TMP}"
    Run Process    bash    -lc    ${cmd}    shell=True
    File Should Exist       ${RESC_TMP}

    # --disable-xwt 避免 GUI，執行腳本後退出；stdout 存到檔案
    ${p}=    Start Process    bash    -lc    "renode --disable-xwt -e \"s @${RESC_TMP}; q\""    shell=True    stdout=${RENODE_LOG}    stderr=STDOUT
    Wait For Process         ${p}    timeout=40s
    File Should Exist        ${RENODE_LOG}

    # 以字串方式抓取 'PTY: /dev/pts/N' 這一行
    ${rc}    ${PTYLINE}=    Run And Return Rc And Output    bash -lc "grep -E 'PTY:\\s+/dev/pts/[0-9]+' ${RENODE_LOG} | tail -n1"
    Run Keyword If    ${rc} != 0    Fail    Could not find 'PTY: /dev/pts/N' in ${RENODE_LOG}
    ${pty}=    Replace String    ${PTYLINE}    PTY:     ${EMPTY}
    ${pty}=    Strip String      ${pty}
    RETURN    ${pty}

*** Test Cases ***
UART heartbeat can be captured from PTY
    [Documentation]    從 Renode 取得 PTY，讀取 2 秒裝置輸出，應包含 XRCE-STUB heartbeat。
    File Should Exist    ${ELF}
    ${PTY}=    Start Renode And Extract PTY
    Run Keyword If    '${PTY}' == ''    Fail    Could not determine PTY path from ${RENODE_LOG}

    # 讀取 PTY 2 秒並寫入檔案
    ${reader}=    Start Process    bash    -lc    "timeout 2s cat ${PTY} > ${UART_LOG}"    shell=True
    Wait For Process    ${reader}    timeout=10s

    File Should Exist    ${UART_LOG}
    ${sz}=    Get File Size    ${UART_LOG}
    Should Be True       ${sz} > 0

    ${output}=    Get File    ${UART_LOG}
    Should Contain       ${output}    XRCE-STUB heartbeat
