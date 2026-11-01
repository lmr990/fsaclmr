#!/usr/bin/env python3
"""Governance phase 1: raw topic/TF inventory from source grep.

Scans src/ for create_publisher( / create_subscription( / create_service(
/ client.call_async( and TF publisher patterns, printing one raw entry per
call site: kind, topic (literal or parameter expression), message/service
type, file:line. Used to reconcile src/TOPICS_AND_NODES.md against the
code; NOT a runtime source of truth — parameter-derived topic names must
be resolved by hand (yaml/launch values noted in the doc).

Usage: python3 scripts/topic_inventory.py [--with-params]
"""

import argparse
import re
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"

# packages that are vendored third-party trees, not project interfaces
SKIP_PKGS = {
    "zed-ros2-wrapper", "zed-ros2-interfaces", "zed-ros2-description",
    "eufs_sim", "TCone-YOLO-main", "QUTMS_Embedded_Common",
}

CALL_RE = re.compile(
    r"(create_publisher|create_subscription|create_service|create_client)"
    r"\(\s*([^,]+),\s*([^,]+),",
    re.DOTALL,
)


def is_string_literal(expr: str) -> bool:
    return expr.strip().startswith(("'", '"')) and not expr.strip().startswith(("' +", '" +'))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--with-params", action="store_true",
                    help="also list parameter-derived topics (default: literals only)")
    args = ap.parse_args()

    rows = []
    for py in sorted(SRC.rglob("*.py")):
        rel = py.relative_to(SRC)
        if rel.parts and rel.parts[0] in SKIP_PKGS:
            continue
        text = py.read_text(errors="replace")
        for m in CALL_RE.finditer(text):
            # uniform signature: (msg_type, topic_name, ...)
            kind, typ, topic = m.group(1), m.group(2).strip(), m.group(3).strip()
            line = text.count("\n", 0, m.start()) + 1
            literal = is_string_literal(topic)
            if literal or args.with_params:
                rows.append((rel, line, kind, topic, typ, "literal" if literal else "param"))

    for rel, line, kind, topic, typ, src in rows:
        print(f"{kind:18s} {topic:42s} {typ:28s} {src:7s} {rel}:{line}")
    print(f"\n# total call sites listed: {len(rows)}")
    lit = [r for r in rows if r[5] == "literal"]
    print(f"# literal call sites: {len(lit)}; distinct literal topics: "
          f"{len(set(r[3] for r in lit))}")


if __name__ == "__main__":
    main()
