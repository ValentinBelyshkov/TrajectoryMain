#!/usr/bin/env python3
"""Detached manager launcher. Spawns system_manager.py in a new session so it
survives the plink/SSH session that invokes this script (start_new_session)."""
import datetime
import os
import subprocess

LOG = "/tmp/manager.log"


def main():
    with open(LOG, "a") as f:
        f.write("\n=== manager daemon spawn %s ===\n" % datetime.datetime.now())

    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source /opt/main/Trajectory/host_colcon_ws/install/setup.bash && "
        "exec python3 /opt/main/Trajectory/TerraSLAM_relay/system_manager.py --reload"
    )
    subprocess.Popen(
        ["bash", "-lc", cmd],
        start_new_session=True,
        cwd="/opt/main/Trajectory",
        stdout=open(LOG, "a"),
        stderr=subprocess.STDOUT,
    )
    print("manager spawn requested (detached)")


if __name__ == "__main__":
    main()
