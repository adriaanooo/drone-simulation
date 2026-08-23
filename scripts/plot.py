import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

df = pd.read_csv("output.csv")

sns.set_style('darkgrid')

fig, axes = plt.subplots(2, 4)
axes.ravel()

sns.lineplot(x=df['Time (s)'], y=df['Target Roll Angle (rad)'], label='Target Roll Angle (rad)', ax=axes[0, 0])
sns.lineplot(x=df['Time (s)'], y=df['Roll Angle (rad)'], label='Roll Angle (rad)', ax=axes[0, 0])
sns.lineplot(x=df['Time (s)'], y=df['Roll Velocity (rad/s)'], label='Roll Velocity (rad/s)', ax=axes[1, 0])
sns.lineplot(x=df['Time (s)'], y=df['Target Pitch Angle (rad)'], label='Target Pitch Angle (rad)', ax=axes[0, 1])
sns.lineplot(x=df['Time (s)'], y=df['Pitch Angle (rad)'], label='Pitch Angle (rad)', ax=axes[0, 1])
sns.lineplot(x=df['Time (s)'], y=df['Pitch Velocity (rad/s)'], label='Pitch Velocity (rad/s)', ax=axes[1, 1])
sns.lineplot(x=df['Time (s)'], y=df['FL Motor RPM'], label='FL Motor RPM', ax=axes[0, 2])
sns.lineplot(x=df['Time (s)'], y=df['FR Motor RPM'], label='FR Motor RPM', ax=axes[1, 2])
sns.lineplot(x=df['Time (s)'], y=df['RL Motor RPM'], label='RL Motor RPM', ax=axes[0, 3])
sns.lineplot(x=df['Time (s)'], y=df['RR Motor RPM'], label='RR Motor RPM', ax=axes[1, 3])

plt.legend()
plt.tight_layout()
plt.show()