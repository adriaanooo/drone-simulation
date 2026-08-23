import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

df = pd.read_csv("output.csv")
df = df.iloc[::1000].iloc[10:].copy()

sns.set_style('darkgrid')

fig, axes = plt.subplots(4, 1)
axes.ravel()

sns.lineplot(x=df['Time (s)'], y=df['Target Roll Rate (deg/s)'], label='Target Roll Angle (deg)', ax=axes[0])
sns.lineplot(x=df['Time (s)'], y=df['Target Pitch Rate (deg/s)'], label='Target Pitch Angle (deg)', ax=axes[0])
sns.lineplot(x=df['Time (s)'], y=df['Target Yaw Rate (deg/s)'], label='Target Yaw Angle (deg)', ax=axes[0])
sns.lineplot(x=df['Time (s)'], y=df['Roll Angle (deg)'], label='Roll Angle (deg)', ax=axes[2])
sns.lineplot(x=df['Time (s)'], y=df['Roll Rate (deg/s)'], label='Roll Velocity (deg/s)', ax=axes[1])
sns.lineplot(x=df['Time (s)'], y=df['Pitch Angle (deg)'], label='Pitch Angle (deg)', ax=axes[2])
sns.lineplot(x=df['Time (s)'], y=df['Pitch Rate (deg/s)'], label='Pitch Velocity (deg/s)', ax=axes[1])
sns.lineplot(x=df['Time (s)'], y=df['Yaw Angle (deg)'], label='Yaw Angle (deg)', ax=axes[2])
sns.lineplot(x=df['Time (s)'], y=df['Yaw Rate (deg/s)'], label='Yaw Velocity (deg/s)', ax=axes[1])
sns.lineplot(x=df['Time (s)'], y=df['FL Motor RPM'], label='FL Motor RPM', ax=axes[3])
sns.lineplot(x=df['Time (s)'], y=df['FR Motor RPM'], label='FR Motor RPM', ax=axes[3])
sns.lineplot(x=df['Time (s)'], y=df['RL Motor RPM'], label='RL Motor RPM', ax=axes[3])
sns.lineplot(x=df['Time (s)'], y=df['RR Motor RPM'], label='RR Motor RPM', ax=axes[3])

plt.legend()
plt.show()