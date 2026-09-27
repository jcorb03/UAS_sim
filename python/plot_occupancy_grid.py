from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

project_dir = Path(__file__).resolve().parent.parent

grid_path = project_dir / "results" / "grid.csv"

bounds_path = project_dir / "results" / "bounds.csv"

bounds_df = pd.read_csv(bounds_path)

df = pd.read_csv(grid_path)

x_values = np.sort(df["X"].unique())
y_values = np.sort(df["Y"].unique())

nx = len(x_values)
ny = len(y_values)

grid = df["Status"].to_numpy().reshape(nx, ny).T

plt.imshow(
    grid,
    origin="lower",
    extent=[
        x_values.min(),
        x_values.max(),
        y_values.min(),
        y_values.max()
    ],
    interpolation="nearest"
)

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("UAS Occupancy Grid")

plt.show()
