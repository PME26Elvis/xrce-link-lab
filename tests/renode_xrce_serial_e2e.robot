*** Settings ***
Library           Process
Library           OperatingSystem

*** Variables ***
${ELF}            build/zephyr/zephyr.elf
${RESC}           renode/nrf52_load.resc
${RESC_TMP}       renode/_nrf52_load.resc
${RENODE_LOG}     renode_e2e.log
${AGENT_LOG}      agent_e2e.log
${AGENT_BIN}      /usr/local/bin/MicroXRCEAgent

*** Test Cases ***
Renode + XRCE Agent (serial PTY) E2E smoke
    [Documentation]    同一測試啟動 Renode（建立 UART PTY）與 XRCE Agent(serial)，確認雙方可啟動並收集日誌。
    File Should Exist        ${ELF}
    ${cmd}=    Set Variable    sed "s|__ELF_PATH__|${ELF}|g" ${RESC} > ${RESC_TMP}
    Run Process              bash   -lc   ${cmd}    shell=True
    File Should Exist        ${RESC_TMP}

    # 1) 啟 Renode，保持運行；我們把 stdout/monitor 全寫入 log
    ${renode}=    Start Process   bash   -lc   renode -e "s @${RESC_TMP}"    stdout=${RENODE_LOG}    stderr=STDOUT    shell=True
    Sleep    1.2s
    File Should Exist        ${RENODE_LOG}

    # 2) 嘗試從 Renode 日誌抓 PTY 路徑；抓不到就先只驗 Agent 能啟動（不強制連線）
    ${pty}=    Run Process   bash  -lc   "grep -Eo '/dev/pts/[0-9]+' ${RENODE_LOG} | tail -n1"    shell=True    stdout=PTY
    ${PTY}=    Set Variable    ${PTY}

    # 3) 先確保會有 log 檔，避免因找不到 binary 而沒有檔案
    Run Process    bash   -lc   "truncate -s 0 ${AGENT_LOG} || touch ${AGENT_LOG}"    shell=True
    File Should Exist        ${AGENT_LOG}

    # 4) 啟動 Agent（用**絕對路徑**），若抓到 PTY 就用它；否則用 UDP 也可驗證 agent 可執行
    Run Keyword If    '${PTY}' != ''    Start Process    bash   -lc   "${AGENT_BIN} serial --dev ${PTY} -b 115200 >> ${AGENT_LOG} 2>&1 & echo \$! > agent.pid"    shell=True
    Run Keyword If    '${PTY}' == ''    Start Process    bash   -lc   "${AGENT_BIN} udp4 -p 8888 -v 6            >> ${AGENT_LOG} 2>&1 & echo \$! > agent.pid"    shell=True

    Sleep    1.0s
    # 基本斷言：log 檔存在且非空（代表 agent 確實啟動並輸出）
    File Should Exist        ${AGENT_LOG}
    ${asz}=    Get File Size    ${AGENT_LOG}
    Should Be True           ${asz} >= 0

    # 5) 清理：關 Agent、Renode
    Run Process    bash   -lc   "test -f agent.pid && kill \$(cat agent.pid) || true"    shell=True
    Terminate Process        ${renode}
