# TeraStation 5400 replacement firmware

This is firmware for the [Buffalo TeraStation 5400](https://www.buffalotech.com/products/terastation-5000n-series) NAS device - I'm using a 5400R 1U rackmount unit.

There are two parts:

- Kernel modules to support the SUGI platform hardware
- Scripts to interface with the kernel-provided fan/power control

The kernel modules are mostly copied from the open source Buffalo Linux kernel, with some minor changes so they compile with more modern kernel sources.

All scripts are rewritten from scratch in Python to avoid any licensing issues (the scripts provided with the device are mostly bash/sh scripts)

Everything here is licensed using GPL.
