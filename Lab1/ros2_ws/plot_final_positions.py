#!/usr/bin/env python3
"""
Parse final_positions.csv (trial,x,y) and render a scatter plot of the bot's
final stopping positions for each trial.
"""

import argparse
import csv
import matplotlib.pyplot as plt


DEFAULT_CSV = "final_positions.csv"


def main():
    parser = argparse.ArgumentParser(description="Plot final positions from final_positions.csv")
    parser.add_argument(
        "csv_file",
        nargs="?",
        default=DEFAULT_CSV,
        help="Path to final_positions.csv (default: %s)" % DEFAULT_CSV,
    )
    args = parser.parse_args()

    trials = []
    x_vals = []
    y_vals = []

    try:
        with open(args.csv_file, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                trials.append(int(row["trial"]))
                x_vals.append(float(row["x"]))
                y_vals.append(float(row["y"]))
    except FileNotFoundError:
        print("File not found: %s (run run_10_trials first)" % args.csv_file)
        return
    except KeyError as e:
        print("CSV must have header: trial,x,y. Error: %s" % e)
        return

    if not trials:
        print("No data in %s" % args.csv_file)
        return

    fig, ax = plt.subplots(1, 1)
    ax.set_aspect("equal")
    scatter = ax.scatter(x_vals, y_vals, c=trials, cmap="viridis", s=80, edgecolors="black")

    # Optional: mark target (0, 0) if near the data range
    ax.axhline(0, color="gray", linestyle="--", linewidth=0.8)
    ax.axvline(0, color="gray", linestyle="--", linewidth=0.8)
    ax.plot(0, 0, "r*", markersize=12, label="Target (0, 0)")

    for i, (xi, yi) in enumerate(zip(x_vals, y_vals)):
        ax.annotate(str(trials[i]), (xi, yi), xytext=(5, 5), textcoords="offset points", fontsize=9)

    plt.colorbar(scatter, ax=ax, label="Trial")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Final stopping positions (%d trials)" % len(trials))
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
