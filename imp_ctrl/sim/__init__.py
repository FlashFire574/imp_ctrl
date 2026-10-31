"""Utilities for plotting controller telemetry."""

import matplotlib.pyplot as plt


def report(log, x_start):
    fig, axes = plt.subplots(2, 1, figsize=(8, 8))
    axes[0].plot(log["t"], [p[0] for p in log["pos"]], label="x")
    axes[0].plot(log["t"], [p[1] for p in log["pos"]], label="y")
    axes[0].plot(log["t"], [p[2] for p in log["pos"]], label="z")
    axes[0].plot(log["t"], [t[0] for t in log["target"]], linestyle="--", label="target x")
    axes[0].legend()
    axes[1].plot(log["t"], log["ori_err"], label="orientation error")
    axes[1].legend()
    plt.show()
