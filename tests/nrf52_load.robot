*** Settings ***
Library           Process
Library           OperatingSystem

*** Variables ***
${RESC}           renode/nrf52_load.resc
${RESC_TMP}       renode/_nrf52_load.resc
${ELF}            build/zephyr/zephyr.elf
${RENODE_LOG}     renode_run.log

*** Test Cases ***
Renode loads nRF52840 ELF and runs shortly
    [Documentation]    確認 Renode 能載入 nRF52840 ELF 並執行 1 秒（不接 Agent）
    File Should Exist        ${ELF}
    ${cmd}=    Set Variable  sed "s|__ELF_PATH__|${ELF}|g" ${RESC} > ${RESC_TMP}
    Run Process             bash  -lc  ${cmd}    shell=True
    File Should Exist        ${RESC_TMP}

    ${p}=    Start Process   bash  -lc  renode -e "s @${RESC_TMP}; q" > ${RENODE_LOG} 2>&1    shell=True
    Wait For Process         ${p}   timeout=30s

    File Should Exist        ${RENODE_LOG}
    ${size}=    Get File Size    ${RENODE_LOG}
    Should Be True    ${size} > 0
