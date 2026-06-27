#!/usr/bin/env python3
"""Run the left side-only LCA test."""

import sys

from lca_side_only import main


if __name__ == "__main__":
    raise SystemExit(main(["left", *sys.argv[1:]]))
