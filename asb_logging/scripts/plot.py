from os import path
import matplotlib.pyplot as plt
import pandas as pd
plt.rcParams['figure.figsize'] = [10, 10]

csv_filepath = "2023-12-15_14-19-50.019315.csv"
# csv_filepath = "2023-12-15_14-24-30.189452.csv"
logs_path = path.expanduser("~/asb_logs/")
d = pd.read_csv(path.join(logs_path, csv_filepath))

plt.plot(d.dt, d.left_motor_velocity_setpoint, '--', color='red', linewidth=1.5, label="left_motor_velocity_setpoint")
plt.plot(d.dt, d.left_motor_velocity, '-', color='red', label="left_motor_velocity")
plt.plot(d.dt, d.right_motor_velocity_setpoint, '--', color='blue', linewidth=1.5, label="right_motor_velocity_setpoint")
plt.plot(d.dt, d.right_motor_velocity, '-', color='blue', label="right_motor_velocity")
plt.legend()
plt.show()
