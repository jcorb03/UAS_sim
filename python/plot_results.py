from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from matplotlib.animation import PillowWriter


project_dir = Path(__file__).resolve().parent.parent
csv_path = project_dir / "results" / "results.csv"

df = pd.read_csv(csv_path)

metadata = dict(title="UAS animation")
writer = PillowWriter(fps=50, metadata=metadata)

fig = plt.figure()



plt.title("UAS Trajectory")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")

min_X = -1000
max_X = 1000
min_Y = -1000
max_Y = 1000
no_uavs = 0

for column in df.columns:

    if "_X" in column:
        min_X = min(min_X, df[column].min() - 10)
        max_X = max(max_X, df[column].max() + 10)
        no_uavs += 1

    if "_Y" in column:
        min_Y = min(min_Y, df[column].min() - 10)
        max_Y = max(max_Y, df[column].max() + 10)



plt.xlim(min_X, max_X)
plt.ylim(min_Y, max_Y)

# Create one line for each UAV
lines = []

for j in range(no_uavs):
    line, = plt.plot([], [], label=f"UAV {j}")
    lines.append(line)

plt.legend()

with writer.saving(fig, "results/UAS.gif", 100):

    for i in range(len(df)):

        for j in range(no_uavs):

            lines[j].set_data(
                df[f"UAV{j}_X"].iloc[:i+1],
                df[f"UAV{j}_Y"].iloc[:i+1]
            )

        writer.grab_frame()