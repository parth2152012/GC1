#!/usr/bin/env python3
import os
from pathlib import Path
import shlex
import sys
import subprocess
import time
from shutil import which

DEFAULT_ARDUPILOT_DIR = str(Path.home() / "ardupilot")
RUST_APP_DIR = Path(__file__).resolve().parent
DRONE_SEPARATION_M = 5
CONTROL_PORT_BASE = 14600


def cleanup():
    print("🧹 Cleaning up existing SITL, MAVProxy, and Rust processes...")
    subprocess.run(["pkill", "-f", "arducopter"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "sim_vehicle.py"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "GC1"], stderr=subprocess.DEVNULL)
    time.sleep(1)


def main():
    cleanup()

    # Suppress the harmless Qt compose-table diagnostic printed by some
    # qterminal/X11 builds. It is unrelated to SITL or the Rust controller.
    os.environ["QT_LOGGING_RULES"] = "qt.xkb.compose.warning=false"

    print("==================================================")
    print("📍 SWARM LOCATION CONFIGURATION")
    print("==================================================")

    default_lat = 19.0760
    default_lon = 72.8777

    ardupilot_input = input(
        f"Enter ArduPilot directory [{DEFAULT_ARDUPILOT_DIR}]: "
    ).strip()
    ardupilot_dir = Path(ardupilot_input or DEFAULT_ARDUPILOT_DIR).expanduser()
    if not (ardupilot_dir / "Tools/autotest/sim_vehicle.py").is_file():
        print(f"❌ ArduPilot sim_vehicle.py not found under {ardupilot_dir}")
        return

    try:
        drone_input = input("Enter number of drones [2]: ").strip()
        drone_count = int(drone_input) if drone_input else 2
        if not 1 <= drone_count <= 20:
            raise ValueError
    except ValueError:
        print("❌ Drone count must be between 1 and 20.")
        return

    try:
        lat_input = input(f"Enter Origin Latitude [{default_lat}]: ").strip()
        lat1 = float(lat_input) if lat_input else default_lat

        lon_input = input(f"Enter Origin Longitude [{default_lon}]: ").strip()
        lon1 = float(lon_input) if lon_input else default_lon
    except ValueError:
        print("❌ Invalid input. Falling back to defaults.")
        lat1, lon1 = default_lat, default_lon

    print(f"\n🚀 Launching {drone_count} drone(s) with {DRONE_SEPARATION_M}m north/south separation.")
    for index in range(drone_count):
        drone_lat = lat1 + (index * DRONE_SEPARATION_M * 0.000009)
        print(f"🚀 Launching Drone {index + 1} at: {drone_lat:.6f}, {lon1:.6f}")
    print("==================================================\n")

    print("🖥️  Opening SITL Drone Instances in dedicated terminals...")

    # Prefer qterminal, with the desktop-provided terminal as a fallback.
    terminal = "qterminal" if which("qterminal") else "x-terminal-emulator"
    if not which(terminal):
        print("❌ No supported graphical terminal was found (qterminal or x-terminal-emulator).")
        return

    for index in range(drone_count):
        drone_lat = lat1 + (index * DRONE_SEPARATION_M * 0.000009)
        telemetry_port = 14550 + (index * 10)
        control_port = CONTROL_PORT_BASE + index
        command = (
            f"cd {shlex.quote(str(ardupilot_dir / 'ArduCopter'))} && "
            f"python3 {shlex.quote(str(ardupilot_dir / 'Tools/autotest/sim_vehicle.py'))} "
            f"-v ArduCopter -I {index} --sysid {index + 1} --console --map "
            f"--out=udp:127.0.0.1:{telemetry_port} "
            f"--out=tcpin:127.0.0.1:{control_port} "
            f"--custom-location={drone_lat},{lon1},0,0 --no-rebuild --wipe-eeprom"
        )
        subprocess.Popen([terminal, "-e", f"bash -c {shlex.quote(command + '; exec bash')}"])

    print("⚡ Starting Rust Swarm Controller...\n")
    os.chdir(RUST_APP_DIR)

    try:
        subprocess.run(["cargo", "run"], env={**os.environ, "GC1_DRONE_COUNT": str(drone_count)})
    except KeyboardInterrupt:
        print("\n🛑 Shutting down swarm system...")
        cleanup()


if __name__ == "__main__":
    main()
