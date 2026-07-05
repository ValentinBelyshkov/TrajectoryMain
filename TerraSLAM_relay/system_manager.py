#!/usr/bin/env python3
"""
TerraSLAM System Manager — entrypoint.

This file is kept as the process entrypoint (invoked directly by
supervisord.conf and entrypoint.sh). The actual implementation lives in
the system_manager_pkg package, split into focused modules of ~400 lines
or less each — see system_manager_pkg/app.py for the module map.
"""
import os

from system_manager_pkg.app import app  # noqa: F401

if __name__ == "__main__":
    import uvicorn

    port = int(os.environ.get("TERRASLAM_PORT", "9000"))
    uvicorn.run(app, host="0.0.0.0", port=port)
