"""Read an existing Gowin timing report; does not invoke FPGA tools."""
import argparse
from html.parser import HTMLParser
from pathlib import Path


class Rows(HTMLParser):
    def __init__(self):
        super().__init__()
        self.rows = []
        self.row = None
        self.cell = None

    def handle_starttag(self, tag, attrs):
        if tag == "tr":
            self.row = []
        elif tag in ("td", "th"):
            self.cell = []

    def handle_data(self, data):
        if self.cell is not None:
            self.cell.append(data)

    def handle_endtag(self, tag):
        if tag in ("td", "th") and self.cell is not None:
            if self.row is not None:
                self.row.append("".join(self.cell).strip())
            self.cell = None
        elif tag == "tr" and self.row is not None:
            self.rows.append(self.row)
            self.row = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--contains", default="")
    args = parser.parse_args()
    rows = Rows()
    rows.feed(args.report.read_text(encoding="utf-8"))
    seen = set()
    for row in rows.rows:
        if len(row) == 9 and row[0].isdigit():
            try:
                slack = float(row[1])
            except ValueError:
                continue
            if slack < 0 and args.contains in " | ".join(row):
                key = tuple(row[1:])
                if key not in seen:
                    seen.add(key)
                    print(" | ".join(row[1:]))


if __name__ == "__main__":
    main()
