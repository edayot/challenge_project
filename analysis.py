

import pandas as pd
import pathlib
import matplotlib.pyplot as plt

df = pd.read_csv(pathlib.Path(__file__).parent/"data.csv")

df.sort_values(by="time",inplace=True)

LIMIT = 192.643000

red_part = df[df["time"]<LIMIT]
green_part = df[(df["time"]>=LIMIT)]

# print the maximum absolute angular velocity in both parts
print(red_part["az"].abs().max(), " max angular velocity in red part")
print(green_part["az"].abs().max(), " max angular velocity in green part")

plt.plot(red_part["time"],red_part["az"],label="Corridor",color="red")
plt.plot(green_part["time"],green_part["az"],label="Line following",color="green")
plt.legend()
plt.xlabel("time")
plt.ylabel("angular velocity")
plt.title("Angular velocity over time")
plt.show()


plt.plot(red_part["time"],red_part["lx"],label="Corridor",color="red")
plt.plot(green_part["time"],green_part["lx"],label="Line following",color="green")
plt.legend()
plt.xlabel("time")
plt.ylabel("linear velocity")
plt.title("Linear velocity over time")
plt.show()

