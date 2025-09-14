*** Settings ***
Library           Process
Library           OperatingSystem

*** Variables ***
${ELF}            build/zephyr/zephyr.elf
${RESC}           renode/nrf52_load.resc
${RESC_TMP}       renode/_nrf52_load_e2e.resc
${RENODE_LOG}     artifacts/renode_agent_serial/renode_e2e.log
${AGENT_LOG}      artifacts/renode_agent_serial/agent_e2e.log

*** Keywords ***
Start Renode In Background And Get PTY
    # 準備 artifacts 目錄與替換後的 resc
    Run Process    bash    -lc    "mkdir -p artifacts/renode_agent_serial"    shell=True
    ${cmd}=    Set Variable    sed "s|__ELF_PATH__|${ELF}|g" ${RESC} > ${RESC_TMP}
    Run Process    bash    -lc    ${cmd}    shell=True
    File Should Exist    ${RESC_TMP}

    # 背景啟 Renode（不 q），讓它跑起來並產生日誌
    ${renode}=    Start Process    bash    -lc    "renode -e 's @${RESC_TMP}; start' > ${RENODE_LOG} 2>&1"    shell=True
    Sleep    1.0s

    # 從 log 擷取 PTY 路徑（重試幾次以等待 log 刷出）
    ${pty}=    Set Variable    ${EMPTY}
    :FOR    ${i}    IN RANGE    1    8
    \    ${pty}=    Run Process    bash    -lc    "grep -Eo '/dev/pts/[0-9]+' ${RENODE_LOG} | tail -n1 || true"    shell=True    stdout=PTY
    \    Run Keyword If    '${PTY}' != ''    Exit For Loop
    \    Sleep    0.5s

    RETURN    ${PTY}    ${renode}

*** Test Cases ***
Renode + XRCE Agent (serial PTY) E2E smoke
    [Documentation]    背景啟 Renode→取得 PTY→啟 XRCE Agent(serial)→驗證 Agent 有輸出→收尾。
    File Should Exist        ${ELF}
    ${PTY}    ${RENODE}=    Start Renode In Background And Get PTY
    Run Keyword If    '${PTY}' == ''    Fail    Could not determine PTY path from Renode log.

    # 啟 Agent 指向該 PTY，留一段時間讓它跑
    ${agent}=    Start Process    bash   -lc   "MicroXRCEAgent serial --dev ${PTY} -b 115200 > ${AGENT_LOG} 2>&1"    shell=True
    Sleep    1.5s

    # 驗證：Agent 有產生日誌（此階段不要求 session 成功，先 smoke）
    File Should Exist    ${AGENT_LOG}
    ${asz}=    Get File Size    ${AGENT_LOG}
    Should Be True       ${asz} > 0

    # 收尾
    Terminate Process    ${agent}
    Terminate Process    ${RENODE}
