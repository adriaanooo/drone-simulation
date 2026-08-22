import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

df = pd.read_csv("output.csv")

sns.set_style('darkgrid')

for var in df.drop(columns=['time']).columns:
    sns.lineplot(x=df['time'], y=df[var], label=var)

plt.legend()
plt.show()