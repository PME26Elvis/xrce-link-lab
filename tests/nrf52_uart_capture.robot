*** Settings ***
Library           Process
Library           OperatingSystem
Library           String

*** Variables ***
${REPO_ROOT}      ${CURDIR}/..
${ELF}            ${REPO_ROOT}/build/zephyr/zephyr.elf
${RESC}           ${REPO_ROOT}/renode/nrf52_load.resc
${RESC_TMP}       ${REPO_ROOT}/renode/_nrf52_uart_capture.resc
${RENODE_LOG}     ${REPO_ROOT}/artifacts/uart_capture/uart_capture.renode.log
${UART_LOG}       ${REPO_ROOT}/artifacts/uart_capture/uart_capture.device.log

*** Keywords ***
Start Renode In Background And Get PTY
    Run Process    bash    -lc    "mkdir -p ${REPO_ROOT}/artifacts/uart_capture"    shell=True
    ${resc_text}=              Get File           ${RESC}
    ${patched}=                Replace String     ${resc_text}    __ELF_PATH__    ${ELF}
    Create File                ${RESC_TMP}        ${patched}
    File Should Exist          ${RESC_TMP}
    ${renode}=    Start Process    bash    -lc    renode -e "s @${RESC_TMP}; start"    stdout=${RENODE_LOG}    stderr=STDOUT    shell=True
    Sleep    1.0s
    ${pty}=    Set Variable    ${EMPTY}
    FOR    ${i}    IN RANGE    1    8
        ${pty}=    Run Process    bash    -lc    "grep -Eo '/dev/pts/[0-9]+' '${RENODE_LOG}' | tail -n1 || true"    shell=True    stdout=PTY
        Run Keyword If    '${PTY}' != ''    Exit For Loop
        Sleep    0.5s
    END
    RETURN    ${PTY}    ${renode}

*** Test Cases ***
UART heartbeat can be captured from PTY
    [Documentation]    背景啟 Renode，並行讀取 PTY 2 秒，應至少看到一次 'XRCE-STUB heartbeat'。
    File Should Exist          ${ELF}
    ${PTY}    ${RENODE}=       Start Renode In Background And Get PTY
    Run Keyword If             '${PTY}' == ''    Fail    Could not determine PTY path from Renode log.

    Run Process    bash    -lc    "timeout 2s cat ${PTY} > '${UART_LOG}'"    shell=True
    File Should Exist          ${UART_LOG}
    ${sz}=    Get File Size    ${UART_LOG}
    Should Be True             ${sz} > 0

    ${hit}=    Run Process     bash  -lc   "grep -c 'XRCE-STUB heartbeat' '${UART_LOG}' || true"    shell=True    stdout=COUNT
    Should Not Be Equal        ${COUNT}    0

    Terminate Process          ${RENODE}
