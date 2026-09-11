from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from matplotlib.animation import PillowWriter


project_dir = Path(__file__).resolve().parent.parent
csv_path = project_dir / "results" / "results.csv"

df = pd.read_csv(csv_path)

metadata = dict(title="UAS animation")
writer = PillowWriter(fps=15, metadata=metadata)

fig = plt.figure()
fig.title("UAS trajectory")
l, = plt.plot([], [], '-k')

plt.xlim(df["X"].min()-10, df["X"].max()+10)
plt.ylim(df["Y"].min()-10, df["Y"].max()+10)

with writer.saving(fig, "results/UAS.gif", 100):
    for i in range(len(df)):
        l.set_data(df["X"].iloc[:i+1], df["Y"].iloc[:i+1])

        writer.grab_frame()