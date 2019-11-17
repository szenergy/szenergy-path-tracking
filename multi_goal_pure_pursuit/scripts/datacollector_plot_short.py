import pandas as pd
import matplotlib.pyplot as plt
import glob, os

#x = pd.read_csv("./data/simple_pure_pursuit.csv")
#x = pd.read_csv("./data/simple_pure_pursuit_long.csv")
#x = pd.read_csv("./data/2019_November_15_01_26_25.csv")

# Circular track 
x = pd.read_csv("./data/2019_November_17_17_59_30_short_speedratio.csv")
x = x.rename(columns={'lateral_error':'lateral_error_speed_ratio'})

x1 = pd.read_csv("./data/2019_November_17_18_02_07_short_speedratio.csv")
x1 = x1.rename(columns={'lateral_error':'lateral_error_speed_ratio'})

#y = pd.read_csv("./data/multi_goal_pure_pursuit.csv")
#y = pd.read_csv("./data/multi_goal_pure_pursuit_long.csv")
#y = pd.read_csv("./data/2019_November_15_01_27_55.csv")
#y = pd.read_csv("./data/2019_November_15_22_57_15.csv")
y = pd.read_csv("./data/2019_November_17_18_16_35_short_multigoal.csv")
y = y.rename(columns={'lateral_error':'lateral_error_multi_goal'})
y1 = pd.read_csv("./data/2019_November_17_18_18_36_short_multigoal.csv")
y1 = y1.rename(columns={'lateral_error':'lateral_error_multi_goal'})

# Long track lane geometry
z = pd.read_csv("./data/2019_November_17_17_43_14_short_lanegeometry.csv")
z = z.rename(columns={'lateral_error':'lateral_error_lane_geometry'})
z1 = pd.read_csv("./data/2019_November_17_17_56_37_short_lanegeometry.csv")
z1 = z1.rename(columns={'lateral_error':'lateral_error_lane_geometry'})


ax = plt.gca()
x['time'] -= x['time'].min()
x1['time'] -= x1['time'].min()
y['time'] -= y['time'].min()
y1['time'] -= y1['time'].min()
z['time'] -= z['time'].min()
z1['time'] -= z1['time'].min()
# Offset
"""
x['time'] += 0.3
x1['time'] += 0.15
y1['time'] += 0.35
"""
x.iloc[200:1050].plot(x='time', y='lateral_error_speed_ratio', ax=ax, color='m')
x1.iloc[200:1050].plot(x='time', y='lateral_error_speed_ratio', ax=ax, color='m')
y.iloc[200:2525].plot(x='time', y='lateral_error_multi_goal', ax=ax, color='orange')
y1.iloc[200:2525].plot(x='time', y='lateral_error_multi_goal', ax=ax, color='orange')
z.iloc[200:1050].plot(x='time', y='lateral_error_lane_geometry', ax=ax, color='blue')
z1.iloc[200:1050].plot(x='time', y='lateral_error_lane_geometry', ax=ax, color='blue')

print("Average lat. error (speed-ratio): {0}".format(x.iloc[200:1050]["lateral_error_speed_ratio"].mean()))
print("Average lat. error (multi-goal): {0}".format(y.iloc[200:2525]["lateral_error_multi_goal"].mean()))
print("Average lat. error (multi-goal): {0}".format(y1.iloc[200:2525]["lateral_error_multi_goal"].mean()))
print("Average lat. error (lane-geometry): {0}".format(z.iloc[200:1050]["lateral_error_lane_geometry"].mean()))
print("Average lat. error (lane-geometry): {0}".format(z1.iloc[200:1050]["lateral_error_lane_geometry"].mean()))
print("Max lat. error (speed-ratio): {0}".format(x.iloc[200:1050]["lateral_error_speed_ratio"].max()))
print("Max lat. error (multi-goal): {0}".format(y.iloc[200:2525]["lateral_error_multi_goal"].max()))
print("Max lat. error (multi-goal): {0}".format(y1.iloc[200:2525]["lateral_error_multi_goal"].max()))
print("Max lat. error (lane-geometry): {0}".format(z.iloc[200:1050]["lateral_error_lane_geometry"].max()))
print("Max lat. error (lane-geometry): {0}".format(z1.iloc[200:1050]["lateral_error_lane_geometry"].max()))
plt.show()