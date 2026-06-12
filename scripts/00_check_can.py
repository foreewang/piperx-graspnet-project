import argparse
import time

import _bootstrap  # noqa: F401
import can


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--channel", default="0")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    args = parser.parse_args()

    configs = can.detect_available_configs(interfaces=["agx_cando"])
    print("detected:", configs)
    if not configs:
        return 1

    bus = can.Bus(
        interface="agx_cando",
        channel=args.channel,
        bitrate=args.bitrate,
        local_loopback=False,
        receive_own_messages=False,
    )
    try:
        deadline = time.time() + args.seconds
        count = 0
        first = None
        while time.time() < deadline:
            msg = bus.recv(0.2)
            if msg is not None:
                first = first or msg
                count += 1
        print("opened:", bus.channel_info)
        print("received:", count)
        if first is not None:
            print("first:", first)
        return 0
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
