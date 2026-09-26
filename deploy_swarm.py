#!/usr/bin/env python3
import os
import sys
import subprocess
import time

ARDUPILOT_DIR = "/home/kali/ardupilot"
RUST_APP_DIR = "/home/kali/GC1"


def cleanup():
    print("🧹 Cleaning up existing SITL, MAVProxy, and Rust processes...")
    subprocess.run(["pkill", "-f", "arducopter"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "sim_vehicle.py"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "GC1"], stderr=subprocess.DEVNULL)
    time.sleep(1)


def main():
    cleanup()

    # Suppress Qt environment warnings in GUI subshells
    os.environ["QT_XKB_CONFIG_ROOT"] = "/usr/share/X11/xkb"

    print("==================================================")
    print("📍 SWARM LOCATION CONFIGURATION")
    print("==================================================")

    default_lat = 19.0760
    default_lon = 72.8777

    try:
        lat_input = input(f"Enter Origin Latitude [{default_lat}]: ").strip()
        lat1 = float(lat_input) if lat_input else default_lat

        lon_input = input(f"Enter Origin Longitude [{default_lon}]: ").strip()
        lon1 = float(lon_input) if lon_input else default_lon
    except ValueError:
        print("❌ Invalid input. Falling back to defaults.")
        lat1, lon1 = default_lat, default_lon

    # Add ~10m offset North for Drone 2
    lat2 = lat1 + (10 * 0.000009)
    lon2 = lon1

    print(f"\n🚀 Launching Drone 1 at: {lat1:.6f}, {lon1:.6f}")
    print(f"🚀 Launching Drone 2 at: {lat2:.6f}, {lon2:.6f} (+10m North)")
    print("==================================================\n")

    cmd_drone1 = (
        f"cd {ARDUPILOT_DIR}/ArduCopter && "
        f"python3 {ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py "
        f"-v ArduCopter -I 0 --console --map "
        f"--out=udp:127.0.0.1:14550 "
        f"--custom-location={lat1},{lon1},0,0 "
        f"--no-rebuild --wipe-eeprom"
    )

    cmd_drone2 = (
        f"cd {ARDUPILOT_DIR}/ArduCopter && "
        f"python3 {ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py "
        f"-v ArduCopter -I 1 --console --map "
        f"--out=udp:127.0.0.1:14560 "
        f"--custom-location={lat2},{lon2},0,0 "
        f"--no-rebuild --wipe-eeprom"
    )

    print("🖥️  Opening SITL Drone Instances in dedicated terminals...")

    # Try spawning in x-terminal-emulator or qterminal
    try:
        subprocess.Popen(["qterminal", "-e", f"bash -c '{cmd_drone1}; exec bash'"])
        subprocess.Popen(["qterminal", "-e", f"bash -c '{cmd_drone2}; exec bash'"])
    except FileNotFoundError:
        # Fallback to standard xterm / default system terminal if qterminal isn't present
        subprocess.Popen(
            ["x-terminal-emulator", "-e", f"bash -c '{cmd_drone1}; exec bash'"]
        )
        subprocess.Popen(
            ["x-terminal-emulator", "-e", f"bash -c '{cmd_drone2}; exec bash'"]
        )

    print("⚡ Starting Rust Swarm Controller...\n")
    os.chdir(RUST_APP_DIR)

    try:
        subprocess.run(["cargo", "run"])
    except KeyboardInterrupt:
        print("\n🛑 Shutting down swarm system...")
        cleanup()


if __name__ == "__main__":
    main()
