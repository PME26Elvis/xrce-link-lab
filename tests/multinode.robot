*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}

*** Test Cases ***
Quark Two-Node Demo Loads And Starts
    # Load our helper script, which includes Renode's Quark C1000 multi-node demo
    Execute Command    include @${CURDIR}/../renode/multinode_quark.resc

    # Assert machines exist (names per official demo: "server" and "client")
    ${machines}=    Execute Command    mach
    Should Contain    ${machines}    server
    Should Contain    ${machines}    client

    # Start emulation and sanity-check monitor commands still work
    Start Emulation
    ${out}=    Execute Command    help
    Should Contain    ${out}    Available commands:
