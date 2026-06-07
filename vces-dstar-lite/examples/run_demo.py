#!/usr/bin/env python3
from vces_dstar_lite.core import run_demo

if __name__ == "__main__":
    run_demo(start=1, goals=[14, 44, 68, 38], seed=11, max_steps=220)
