*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}

*** Test Cases ***
Quark Two-Node Demo Loads And Starts
    # 直接載入 Renode 內建多節點腳本（不依賴 repo 內自製 .resc）
    Execute Command    include @scripts/many-nodes/quark-c1000-zephyr/demo.resc

    # 斷言兩台機器存在（官方示例即為 server / client）
    ${machines}=    Execute Command    mach
    Should Contain    ${machines}    server
    Should Contain    ${machines}    client

    # 啟動模擬，並確認 Monitor 指令可用
    Start Emulation
    ${out}=    Execute Command    help
    Should Contain    ${out}    Available commands:
