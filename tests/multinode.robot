*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}

*** Test Cases ***
BLE Two-Node Demo Should Load
    # Load our helper script, which includes Renode's built-in 2-node BLE demo
    Execute Command    include @${CURDIR}/../renode/multinode_ble.resc
    Start Emulation
    ${out}=    Execute Command    help
    Should Contain    ${out}    Available commands:
