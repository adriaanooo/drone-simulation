import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

df = pd.read_csv("output.csv")
df = df.iloc[::100].copy()

sns.set_style('darkgrid')

fig, axes = plt.subplots(4, 2)
axes.ravel()

sns.lineplot(x=df['Time (s)'], y=df['Target Roll Rate (deg/s)'], label='Target Roll Rate (deg/s)', ax=axes[0, 0])
sns.lineplot(x=df['Time (s)'], y=df['Target Pitch Rate (deg/s)'], label='Target Pitch Rate (deg/s)', ax=axes[0, 0])
sns.lineplot(x=df['Time (s)'], y=df['Target Yaw Rate (deg/s)'], label='Target Yaw Rate (deg/s)', ax=axes[0, 0])
sns.lineplot(x=df['Time (s)'], y=df['Roll Angle (deg)'], label='Roll Angle (deg)', ax=axes[2, 0])
sns.lineplot(x=df['Time (s)'], y=df['Roll Rate (deg/s)'], label='Roll Velocity (deg/s)', ax=axes[1, 0])
sns.lineplot(x=df['Time (s)'], y=df['Pitch Angle (deg)'], label='Pitch Angle (deg)', ax=axes[2, 0])
sns.lineplot(x=df['Time (s)'], y=df['Pitch Rate (deg/s)'], label='Pitch Velocity (deg/s)', ax=axes[1, 0])
sns.lineplot(x=df['Time (s)'], y=df['Yaw Angle (deg)'], label='Yaw Angle (deg)', ax=axes[2, 0])
sns.lineplot(x=df['Time (s)'], y=df['Yaw Rate (deg/s)'], label='Yaw Velocity (deg/s)', ax=axes[1, 0])
sns.lineplot(x=df['Time (s)'], y=df['FL Motor RPM'], label='FL Motor RPM', ax=axes[3, 0])
sns.lineplot(x=df['Time (s)'], y=df['FR Motor RPM'], label='FR Motor RPM', ax=axes[3, 0])
sns.lineplot(x=df['Time (s)'], y=df['RL Motor RPM'], label='RL Motor RPM', ax=axes[3, 0])
sns.lineplot(x=df['Time (s)'], y=df['RR Motor RPM'], label='RR Motor RPM', ax=axes[3, 0])
sns.lineplot(x=df['Time (s)'], y=df['PID Roll'], label='PID Roll', ax=axes[0, 1])
sns.lineplot(x=df['Time (s)'], y=df['PID Pitch'], label='PID Pitch', ax=axes[1, 1])
sns.lineplot(x=df['Time (s)'], y=df['PID Yaw'], label='PID Yaw', ax=axes[2, 1])

axes[0, 0].set_ylabel('Angular Rate Target (deg/s)')
axes[1, 0].set_ylabel('Angular Rate (deg/s)')
axes[2, 0].set_ylabel('Angle (deg)')
axes[3, 0].set_ylabel('Motor RPM')

axes[3, 1].set_visible(False)

for ax in axes.flat:
    if ax.get_visible() and ax.get_legend_handles_labels()[0]:
        ax.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0, fontsize=9)

fig.subplots_adjust(wspace=0.55, hspace=0.5, left=0.08, right=0.82, top=0.95, bottom=0.08)

plt.show()
