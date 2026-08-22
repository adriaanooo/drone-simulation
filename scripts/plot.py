import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

df = pd.read_csv("output.csv")
df = df.iloc[::10, :]

sns.set_style('darkgrid')

nrows = len(df.columns) - 1
ncols = 1

fig, axes = plt.subplots(nrows, ncols)

for idx, var in enumerate(df.drop(columns=['time']).columns):
    sns.lineplot(x=df['time'], y=df[var], label=var, ax=axes[idx])

plt.legend()
plt.show()