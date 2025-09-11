# XRCE-Link-Lab
Reproducible micro-ROS (XRCE-DDS) link lab: multi-transport (Serial/UDP/TCP), fault-injectable Renode setups, protocol-level metrics.

## What works now
- CI smoke test with Renode + Robot Framework (no firmware needed).
- Build eProsima Micro XRCE-DDS Agent in CI (CLI sanity check).

## Roadmap (short)
- Add Renode multi-node scripts and UART/UDP topologies.
- Bring Zephyr + micro-ROS firmware (sensor/actuator) and connect to Agent.
- Add fault profiles (delay/loss/bandwidth) + metrics export.
- XRCE trace tool for HB/ACKNACK/retrans stats.

References:
- Renode GitHub Action & Robot keywords.  
- XRCE Agent: UDP/TCP/Serial + **multiserial** support.  
- micro-ROS + Zephyr module / Humble Docker.
