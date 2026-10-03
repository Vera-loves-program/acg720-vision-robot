#!/usr/bin/env python3
"""Package only the R3 troubleshooting patch and guide; no FPGA/UI update."""
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DESTINATION = ROOT.parent / "deliverables"
GUIDE = "香橙派收图排查_Flash固化与SSH操作.md"


def main() -> None:
    DESTINATION.mkdir(parents=True, exist_ok=True)
    archive = DESTINATION / "acg720-support-20261004.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
        for source, destination in (
            ("pc/udp_probe.py", "pc/udp_probe.py"),
            ("tools/fix_orangepi_apt.py", "tools/fix_orangepi_apt.py"),
            ("docs/" + GUIDE, GUIDE),
        ):
            bundle.write(ROOT / source, "acg720_support_20261004/" + destination)
    (DESTINATION / GUIDE).write_bytes((ROOT / "docs" / GUIDE).read_bytes())
    print(archive)
    print(DESTINATION / GUIDE)


if __name__ == "__main__":
    main()
