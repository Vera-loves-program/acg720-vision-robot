#!/usr/bin/env python3
"""Preview, or back up and switch Ubuntu Ports URLs to Aliyun HTTPS on Orange Pi.

Only Ubuntu 22.04 arm64 is supported. Vendor repositories and suite/component
settings are preserved. This does not install packages or disable verification.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from urllib.parse import urlsplit

ALI = "https://mirrors.aliyun.com/ubuntu-ports/"


def replace_ports_urls(text: str) -> str:
    def replace(match: re.Match) -> str:
        url = match.group(0)
        parsed = urlsplit(url)
        if (parsed.path.rstrip("/") == "/ubuntu-ports"
                and not parsed.username and not parsed.password
                and not parsed.query and not parsed.fragment):
            return ALI
        return url

    lines = []
    for line in text.splitlines(keepends=True):
        stripped = line.lstrip()
        if (stripped.startswith("deb ") or stripped.startswith("deb-src ")
                or stripped.startswith("URIs:")):
            # Keep trailing comments exactly as they were.
            content, separator, comment = line.partition("#")
            line = re.sub(r"https?://[^\s]+", replace, content) + separator + comment
        lines.append(line)
    return "".join(lines)


def check_system() -> None:
    if os.name != "posix":
        raise RuntimeError("Run on Orange Pi Linux, not Windows.")
    release = {}
    for line in Path("/etc/os-release").read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            release[key] = value.strip('"')
    arch = subprocess.check_output(["dpkg", "--print-architecture"], text=True).strip()
    if release.get("ID") != "ubuntu" or release.get("VERSION_ID") != "22.04" or arch != "arm64":
        raise RuntimeError("Expected Ubuntu 22.04 arm64; no files changed.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="Back up originals, then apply changes using sudo.")
    args = parser.parse_args()
    check_system()
    root = Path("/etc/apt")
    files = [root / "sources.list"]
    files += sorted((root / "sources.list.d").glob("*.list"))
    files += sorted((root / "sources.list.d").glob("*.sources"))
    changes = []
    for path in files:
        if not path.exists():
            continue
        if path.is_symlink():
            raise RuntimeError(f"Source file is a symlink; review manually: {path}")
        before = path.read_text(encoding="utf-8")
        after = replace_ports_urls(before)
        if before != after:
            changes.append((path, before, after))
            print(f"Will update Ubuntu Ports URLs in {path}")
    if not changes:
        print("No changes needed or no matching Ubuntu Ports URLs found. Nothing written.")
        print("If APT still fails, inspect its error and active source file; do not delete vendor repositories.")
        return 0
    print(f"Destination: {ALI}; affected files: {len(changes)}")
    if not args.apply:
        print("PREVIEW ONLY. Run this script with sudo and --apply to back up and change these files.")
        return 0
    if os.geteuid() != 0:
        raise RuntimeError("Use sudo python3 tools/fix_orangepi_apt.py --apply")
    backup = Path("/var/backups") / ("acg720-apt-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    # Finish every backup before the first mutation.
    for path, _, _ in changes:
        saved = backup / path.relative_to(root)
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, saved)
    print(f"Original files backed up to: {backup}")
    for path, _, after in changes:
        mode = path.stat().st_mode & 0o777
        fd, name = tempfile.mkstemp(prefix=".acg720-", dir=path.parent)
        temporary = Path(name)
        try:
            with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
                stream.write(after)
            os.chmod(temporary, mode)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
    print("Done. Run: sudo apt clean, then sudo apt update (separate commands).")
    print(f"Restore originals if needed: sudo cp -a {backup}/. /etc/apt/")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        raise SystemExit(str(exc))
