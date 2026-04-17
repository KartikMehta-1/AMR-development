# PicoScope / PicoSDK Setup (Linux)

This guide sets up PicoScope software and driver packages on Ubuntu/Debian Linux.

## 1) Scope
- Target OS: Ubuntu 22.04 (Jammy) and similar Debian-based systems.
- Installs PicoTech apt repository.
- Installs `picoscope` package by default (includes app and required drivers).

## 2) One-command helper
From repo root:

```bash
bash Workspace/sensor_test/tools/setup_picoscope_linux.sh stable
```

For early-access channel:

```bash
bash Workspace/sensor_test/tools/setup_picoscope_linux.sh ea
```

Install specific driver packages instead of full GUI app:

```bash
bash Workspace/sensor_test/tools/setup_picoscope_linux.sh stable libps6000a
```

## 3) Verify installation
```bash
which picoscope || true
apt-cache policy picoscope | head
apt-cache search '^libps[0-9]' | head -n 20
```

## 4) Notes
- The script uses `sudo` for system package changes.
- If your user cannot run non-interactive sudo, run it manually and enter password when prompted.
- Replug scope hardware after first driver install.

## 5) SDK usage hint
After packages are installed, library files are typically available under `/opt/picoscope` and/or system library paths depending on package.
