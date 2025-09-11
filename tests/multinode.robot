*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}

*** Test Cases ***
STM32F4 Single-Node Demo Loads And Starts
    # 使用官方文檔保證存在的單節點示例腳本
    Execute Command    include @scripts/single-node/stm32f4_discovery.resc

    # 啟動模擬並檢查 Monitor 正常工作
    Start Emulation
    ${out}=    Execute Command    help
    Should Contain    ${out}    Available commands:
