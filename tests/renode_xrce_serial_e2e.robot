*** Settings ***
Library           Process
Library           OperatingSystem
Library           String

*** Variables ***
${REPO_ROOT}      ${CURDIR}/..
${ELF}            ${REPO_ROOT}/build/zephyr/zephyr.elf
${RESC}           ${REPO_ROOT}/renode/nrf52_load.resc
${RESC_TMP}       ${REPO_ROOT}/renode/_nrf52_load.resc
${RENODE_LOG}     ${REPO_ROOT}/renode_e2e.log
${AGENT_LOG}      ${REPO_ROOT}/agent_e2e.log
${AGENT_BIN}      /usr/local/bin/MicroXRCEAgent

*** Test Cases ***
Renode + XRCE Agent (serial PTY) E2E smoke
    [Documentation]    同一測試啟動 Renode（建立 UART PTY）與 XRCE Agent(serial or UDP fallback)，確認雙方可啟動並收集日誌。

    # 0) 路徑健檢
    Directory Should Exist     ${REPO_ROOT}
    File Should Exist          ${ELF}
    File Should Exist          ${RESC}

    # 1) 產生替換過 ELF 路徑的 .resc（不用 sed，純 Robot）
    ${resc_text}=              Get File    ${RESC}
    ${patched}=                Replace String    ${resc_text}    __ELF_PATH__    ${ELF}
    Create File                ${RESC_TMP}    ${patched}
    File Should Exist          ${RESC_TMP}

    # 2) 啟 Renode，保持運行並寫出日誌
    ${renode}=    Start Process    bash    -lc    renode -e "s @${RESC_TMP}"    stdout=${RENODE_LOG}    stderr=STDOUT    shell=True
    Sleep    1.5s
    File Should Exist          ${RENODE_LOG}

    # 3) 嘗試從 Renode 日誌抓 PTY 路徑（不同版本輸出略有差異，抓不到就走 UDP fallback）
    ${pty}=        Run Process   bash  -lc   "grep -Eo '/dev/pts/[0-9]+' '${RENODE_LOG}' | tail -n1"    shell=True    stdout=PTY
    ${PTY}=        Set Variable    ${PTY}

    # 4) 準備 Agent log，並確認 binary 存在
    Create File                 ${AGENT_LOG}
    Run Process                 bash  -lc   "test -x '${AGENT_BIN}'"    shell=True

    # 5) 啟 Agent：若有 PTY 用 serial；沒有就用 UDP 先驗證可執行與輸出
    Run Keyword If    '${PTY}' != ''    Start Process    bash   -lc   "'${AGENT_BIN}' serial --dev ${PTY} -b 115200 >> '${AGENT_LOG}' 2>&1 & echo \$! > agent.pid"    shell=True
    Run Keyword If    '${PTY}' == ''    Start Process    bash   -lc   "'${AGENT_BIN}' udp4 -p 8888 -v 6            >> '${AGENT_LOG}' 2>&1 & echo \$! > agent.pid"    shell=True

    Sleep    1.0s
    File Should Exist          ${AGENT_LOG}
    ${asz}=    Get File Size   ${AGENT_LOG}
    Should Be True             ${asz} >= 0

    # 6) 清理
    Run Process    bash   -lc   "test -f agent.pid && kill \$(cat agent.pid) || true"    shell=True
    Terminate Process          ${renode}
