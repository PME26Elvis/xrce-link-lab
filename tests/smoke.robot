*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}

*** Test Cases ***
Renode Help Should Work
    ${out}=    Execute Command    help
    Should Contain    ${out}    Available commands:
