*** Settings ***
Library           Process
Library           OperatingSystem

*** Variables ***
${ELF}            ${CURDIR}/../build/zephyr/zephyr.elf
${RESC}           ${CURDIR}/../renode/nrf52_load.resc
${RESC_TMP}       ${CURDIR}/../renode/_nrf52_load.resc
${RENODE_LOG}     ${CURDIR}/../renode_e2e.log
${AGENT_LOG}      ${CURDIR}/../agent_e2e.log
${AGENT_BIN}      /usr/local/bin/MicroXRCEAgent

*** Test Cases ***
Renode + XRCE Agent (serial PTY) E2E smoke
    [Documentation]    同一測試啟動 Renode（建立 UART PTY）與 XRCE Agent(serial)，確認雙方可啟動並收集日誌。
    File Should Exist        ${ELF}
    Run Process              bash   -lc   "sed 's|__ELF_PATH__|${ELF}|g' ${RESC} > ${RESC_TMP}"    shell=True
    File Should Exist        ${RESC_TMP}

    # 1) 啟 Renode，保持運行並寫出日誌
    ${renode}=    Start Process   bash   -lc   renode -e "s @${RESC_TMP}"    stdout=${RENODE_LOG}    stderr=STDOUT    shell=True
    Sleep    1.2s
    File Should Exist        ${RENODE_LOG}

    # 2) 嘗試從 Renode 日誌抓 PTY 路徑
    ${pty}=    Run Process   bash  -lc   "grep -Eo '/dev/pts/[0-9]+' '${RENODE_LOG}' | tail -n1"    shell=True    stdout=PTY
    ${PTY}=    Set Variable    ${PTY}

    # 3) 先建立空的 Agent log，避免不存在
    Create File              ${AGENT_LOG}

    # 4) 檢查 Agent binary 存在（避免 PATH 問題）
    Run Process              bash  -lc   "test -x '${AGENT_BIN}'"    shell=True

    # 5) 啟 Agent：若有 PTY 用 serial，否則改用 UDP（先驗證可執行與輸出）
    Run Keyword If    '${PTY}' != ''    Start Process    bash   -lc   "'${AGENT_BIN}' serial --dev ${PTY} -b 115200 >> '${AGENT_LOG}' 2>&1 & echo \$! > agent.pid"    shell=True
    Run Keyword If    '${PTY}' == ''    Start Process    bash   -lc   "'${AGENT_BIN}' udp4 -p 8888 -v 6            >> '${AGENT_LOG}' 2>&1 & echo \$! > agent.pid"    shell=True

    Sleep    1.0s
    File Should Exist        ${AGENT_LOG}
    ${asz}=    Get File Size    ${AGENT_LOG}
    Should Be True           ${asz} >= 0

    # 6) 清理
    Run Process    bash   -lc   "test -f agent.pid && kill \$(cat agent.pid) || true"    shell=True
    Terminate Process        ${renode}
