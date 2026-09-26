from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

project_dir = Path(__file__).resolve().parent.parent

circles_path = project_dir / "results" / "circles.csv"
rectangles_path = project_dir / "results" / "rectangles.csv"
bounds_path = project_dir / "results" / "bounds.csv"
path_filepath = project_dir / "results" / "path.csv"
checkpoints_path = project_dir / "results" / "checkpoints.csv"
trajectory_path = project_dir / "results" / "results.csv"

circles_df = pd.read_csv(circles_path)
rectangles_df = pd.read_csv(rectangles_path)
bounds_df = pd.read_csv(bounds_path)
path_df = pd.read_csv(path_filepath)
checkpoints_df = pd.read_csv(checkpoints_path)
trajectory_df = pd.read_csv(trajectory_path)

from matplotlib.patches import Rectangle, Circle

#define Matplotlib figure and axis
fig, ax = plt.subplots()

# Set plot bounds
ax.set_xlim(
    bounds_df["MinX"].iloc[0],
    bounds_df["MaxX"].iloc[0]
)

ax.set_ylim(
    bounds_df["MinY"].iloc[0],
    bounds_df["MaxY"].iloc[0]
)

#add rectangle to plot
for _,row in rectangles_df.iterrows():
	height = row["MaxY"] - row["MinY"]
	width = row["MaxX"] - row["MinX"]

	ax.add_patch(
        Rectangle(
            (row["MinX"], row["MinY"]),
            width,
            height
        )
    )

#add circles obstacles to plot
for _,row in circles_df.iterrows():

	ax.add_patch(
        Circle(
            (row["X"], row["Y"]),
            row["Radius"]
        )
    )

#add circles obstacles to plot
for _,row in circles_df.iterrows():

	ax.add_patch(
        Circle(
            (row["X"], row["Y"]),
            row["Radius"]
        )
    )

for _,row in checkpoints_df.iterrows():

	ax.add_patch(
        Circle(
            (row["X"], row["Y"]),
            1,
            color = "red"
        )
    )

#add planned path to plot
ax.plot(path_df["X"],path_df["Y"], color = "green", label = "path")
ax.plot(trajectory_df["UAV0_X"],trajectory_df["UAV0_Y"], color = "orange", label = "trajectory")
ax.legend()
# Keep x and y scales equal
ax.set_aspect("equal")

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
#display plot
plt.show()