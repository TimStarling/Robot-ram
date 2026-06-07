<div align="right">

[中文](README.md) | [English](README_EN.md)

</div>

# Robot RAM

This repository contains design notes and reference materials for an upper-computer driver program for a Juxie robotic arm based on CANopen and the DS402 sub-protocol.

## Target Platform

The target runtime platform is Orange Pi 5:

- Architecture: AArch64
- CPU: Cortex-A55 / Cortex-A76 class cores
- Memory: 16 GB

Debugging can be performed on x86 platforms running Windows 10 or Ubuntu 22.04.

## Development Standards

- Language: C++17
- Comments: Doxygen-style comments
- Encoding: UTF-8
- External library interface: C-style interface
- Possible output forms: Pybind11 library, C++ library, or executable

## Design Direction

The driver reads CSV files to reproduce robotic-arm motion sequences. The system may later support kinematics, teaching, and logging of transmitted and received CAN frames.

The robotic arm uses a 1 Mbps CAN bus. A six-axis arm normally has six motors on the same bus, connected to the host through a serial-to-CAN adapter. Transmission should be single-threaded to avoid bus conflicts and keep the arm safe.
