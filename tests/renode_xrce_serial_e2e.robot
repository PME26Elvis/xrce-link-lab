*** Settings ***
Library           Process
Library           OperatingSystem
Library           String

*** Variables ***
${REPO_ROOT}      ${CURDIR}/..
${ELF}            ${REPO_ROOT}/build/zephyr/zephyr.elf
${RESC}           ${REPO_ROOT}/renode/nrf52_load.resc
${RESC_TMP}       ${OUTPUT_DIR}/_nrf52_load_e2e.resc
${RENODE_LOG}     ${OUTPUT_DIR}/renode_e2e.log
${AGENT_LOG}      ${OUTPUT_DIR}/agent_e2e.log
${AGENT_BIN}      /usr/local/bin/MicroXRCEAgent

*** Keywords ***
Start Renode In Background And Get PTY
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
Renode + XRCE Agent (serial PTY) E2E smoke
    Directory Should Exist     ${REPO_ROOT}
    File Should Exist          ${ELF}
    File Should Exist          ${RESC}
    Run Process                bash  -lc   "test -x '${AGENT_BIN}'"    shell=True

    ${PTY}    ${RENODE}=       Start Renode In Background And Get PTY
    Run Keyword If             '${PTY}' == ''    Fail    Could not determine PTY path from Renode log.

    ${agent}=    Start Process    bash   -lc   "'${AGENT_BIN}' serial --dev ${PTY} -b 115200 >> '${AGENT_LOG}' 2>&1"    shell=True
    Sleep    1.5s
    File Should Exist          ${AGENT_LOG}
    ${asz}=    Get File Size    ${AGENT_LOG}
    Should Be True             ${asz} >= 0

    Terminate Process          ${agent}
    Terminate Process          ${RENODE}
