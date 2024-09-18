# Reimporting necessary libraries after reset
import pandas as pd
import matplotlib.pyplot as plt

# Original data from the user
x = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
y = [275.1, 289.5, 303.0, 314.7, 324.5, 336.4, 349.7, 359.3, 372.0, 386.3]
x_percent = [i * 100 for i in x]  # Convert x values to percentages

# Simulated wheel RPM vs throttle data (as a stand-in for the CSV file)
data = pd.read_csv('build/wheel_rpm_vs_throttle.csv')

# Grouping the data by throttle and calculating mean and standard deviation
grouped_data = data.groupby('Throttle').apply(lambda x: x.tail(10).mean())

# Extract the means and standard deviations for plotting
throttle_values = grouped_data.index
mean_left_front = grouped_data['Left Front RPM']
mean_right_front = grouped_data['Right Front RPM']
mean_left_rear = grouped_data['Left Rear RPM']
mean_right_rear = grouped_data['Right Rear RPM']

# Create the scatter plot with error bars
plt.figure(figsize=(10, 6))

# Plot with error bars for each wheel
plt.plot(throttle_values, mean_left_front, marker='o', linestyle='-', label='ART DT Wheel', color='r')

# Overlaying the previous line plot (Measured ART RPM data)
plt.plot(x, y, marker='o', linestyle='-', color='black', label='Measured ART RPM (Real)')

# Customize the plot
plt.title('Wheel RPM Sim-to-Real Comparison')
plt.xlabel('Throttle %')
plt.ylabel('RPM')
plt.legend()
plt.grid(True)

# Save the plot as an image file
plt.savefig('wheel_rpm_vs_throttle_error_bars_plot.png')

print("Error bar plot saved as 'wheel_rpm_vs_throttle_error_bars_plot.png'")