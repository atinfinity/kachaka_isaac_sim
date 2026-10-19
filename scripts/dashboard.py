"""Web dashboard for the simulator (or a real Kachaka) over kachaka-api.

    uv run --no-project --with kachaka-api==3.17.5.0 python scripts/dashboard.py
    # then open http://127.0.0.1:8080

Off unless started; it needs no ROS, only the gRPC API (--target).
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kachaka_api import KachakaApiClient

from kachaka_isaac_sim.dashboard import serve


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--target", default="127.0.0.1:26400", help="kachaka-api gRPC address")
    ap.add_argument("--host", default="127.0.0.1", help="address to serve on (0.0.0.0 for other machines)")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    server = serve(KachakaApiClient(args.target), args.host, args.port)
    print(f"[dashboard] http://{args.host}:{args.port}  (kachaka-api {args.target})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
