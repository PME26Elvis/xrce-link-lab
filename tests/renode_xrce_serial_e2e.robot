*** Settings ***
Library           Process
Library           OperatingSystem

*** Variables ***
${ELF}            build/zephyr/zephyr.elf
${RESC}           renode/nrf52_load.resc
${RESC_TMP}       renode/_nrf52_load.resc
${RENODE_LOG}     renode_e2e.log
${AGENT_LOG}      agent_e2e.log

*** Test Cases ***
Renode + XRCE Agent (serial PTY) E2E smoke
    [Documentation]    在同一支測試中啟動 Renode（建立 UART PTY）與 XRCE Agent(serial)，確認雙方皆能啟動，並收集日誌。
    File Should Exist        ${ELF}
    ${cmd}=    Set Variable  sed "s|@{ELF_PATH}|${ELF}|g" ${RESC} > ${RESC_TMP}
    Run Process             bash  -lc  ${cmd}    shell=True
    File Should Exist        ${RESC_TMP}

    # 1) 啟 Renode，讓它載入 ELF 並建立 PTY；讓它跑一下再退出
    ${renode}=    Start Process   bash  -lc   renode -e "s @${RESC_TMP}; q" > ${RENODE_LOG} 2>&1    shell=True
    Wait For Process             ${renode}    timeout=40s
    File Should Exist            ${RENODE_LOG}
    ${sz}=    Get File Size      ${RENODE_LOG}
    Should Be True               ${sz} > 0

    # 2) 從 Renode log 嘗試抓取 PTY 路徑（若抓不到就用 socat 方案）
    ${pty}=    Run Process   bash  -lc   "grep -Eo '/dev/pts/[0-9]+' ${RENODE_LOG} | tail -n1"    shell=True    stdout=PTY
    ${PTY}=    Set Variable    ${PTY}

    Run Keyword If    '${PTY}' == ''    Log    "No PTY path in log; will bridge via socat"

    # 3a) 如果抓到 PTY 路徑：直接啟 Agent 指向該 PTY
    Run Keyword If    '${PTY}' != ''    Start Process    bash   -lc   "MicroXRCEAgent serial --dev ${PTY} -b 115200 > ${AGENT_LOG} 2>&1 & echo \$! > agent.pid"    shell=True
    # 3b) 如果抓不到：用 socat 建一個固定連結，再讓 Agent 對這個固定連結啟動
    Run Keyword If    '${PTY}' == ''    Run Process   bash  -lc   "socat -d -d pty,raw,echo=0,link=/tmp/xrce_uart pty,raw,echo=0 & echo \$! > socat.pid"    shell=True
    Run Keyword If    '${PTY}' == ''    Sleep    0.5s
    Run Keyword If    '${PTY}' == ''    Start Process    bash   -lc   "MicroXRCEAgent serial --dev /tmp/xrce_uart -b 115200 > ${AGENT_LOG} 2>&1 & echo \$! > agent.pid"    shell=True

    Sleep    1.0s
    File Should Exist    ${AGENT_LOG}
    ${asz}=    Get File Size    ${AGENT_LOG}
    Should Be True       ${asz} > 0

    # 清理（如果有 socat/agent pid）
    Run Process    bash   -lc   "test -f agent.pid && kill \$(cat agent.pid) || true"    shell=True
    Run Process    bash   -lc   "test -f socat.pid && kill \$(cat socat.pid) || true"    shell=True
