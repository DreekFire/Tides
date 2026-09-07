#!/usr/bin/env python3
"""Download and verify the official AtsuLuaEditor Lua API documentation."""

from hashlib import sha256
from pathlib import Path
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "docs" / "vendor" / "AtsuLuaEditor-LuaDocs.cs"
URL = "https://git.sr.ht/~alisa/FtD-AtsuLuaEditor/blob/master/src/LuaDocs.cs"
EXPECTED_SHA256 = "4651f7d8b764455b9f15a9203a344d74d31aff2f4b45966b771885b6b876490d"


def main() -> int:
    with urlopen(URL, timeout=60) as response:
        payload = response.read()
    digest = sha256(payload).hexdigest()
    if digest != EXPECTED_SHA256:
        raise RuntimeError(
            f"unexpected AtsuLuaEditor documentation hash: {digest}"
        )
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    TARGET.write_bytes(payload)
    print(f"{TARGET} ({len(payload)} bytes, sha256={digest})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
