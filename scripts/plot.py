import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

df = pd.read_csv("output.csv")

sns.set_style('darkgrid')

fig, axes = plt.subplots(4, 1)
axes.ravel()

sns.lineplot(x=df['Time (s)'], y=df['Target Roll Angle (rad)'], label='Target Roll Angle (rad)', ax=axes[0])
sns.lineplot(x=df['Time (s)'], y=df['Target Pitch Angle (rad)'], label='Target Pitch Angle (rad)', ax=axes[0])
sns.lineplot(x=df['Time (s)'], y=df['Target Yaw Angle (rad)'], label='Target Yaw Angle (rad)', ax=axes[0])
sns.lineplot(x=df['Time (s)'], y=df['Roll Angle (rad)'], label='Roll Angle (rad)', ax=axes[1])
sns.lineplot(x=df['Time (s)'], y=df['Roll Velocity (rad/s)'], label='Roll Velocity (rad/s)', ax=axes[2])
sns.lineplot(x=df['Time (s)'], y=df['Pitch Angle (rad)'], label='Pitch Angle (rad)', ax=axes[1])
sns.lineplot(x=df['Time (s)'], y=df['Pitch Velocity (rad/s)'], label='Pitch Velocity (rad/s)', ax=axes[2])
sns.lineplot(x=df['Time (s)'], y=df['Yaw Angle (rad)'], label='Yaw Angle (rad)', ax=axes[1])
sns.lineplot(x=df['Time (s)'], y=df['Yaw Velocity (rad/s)'], label='Yaw Velocity (rad/s)', ax=axes[2])
sns.lineplot(x=df['Time (s)'], y=df['FL Motor RPM'], label='FL Motor RPM', ax=axes[3])
sns.lineplot(x=df['Time (s)'], y=df['FR Motor RPM'], label='FR Motor RPM', ax=axes[3])
sns.lineplot(x=df['Time (s)'], y=df['RL Motor RPM'], label='RL Motor RPM', ax=axes[3])
sns.lineplot(x=df['Time (s)'], y=df['RR Motor RPM'], label='RR Motor RPM', ax=axes[3])

plt.legend()
plt.show()