#!/usr/bin/env bash
# sim_vehicle starts this helper in the background. Keep its SITL binary
# inside the launcher's process group instead of opening another terminal.
exec "$@"
