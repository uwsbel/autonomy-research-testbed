import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a DataFrame
data = pd.read_csv('build/wheel_rpm_vs_throttle.csv')

# Group the data by throttle value and calculate the mean and standard deviation for each group
grouped_data = data.groupby('Throttle').agg(['mean', 'std'])

# Extract the means and standard deviations for plotting
throttle_values = grouped_data.index
mean_left_front = grouped_data[('Left Front RPM', 'mean')]
std_left_front = grouped_data[('Left Front RPM', 'std')]

mean_right_front = grouped_data[('Right Front RPM', 'mean')]
std_right_front = grouped_data[('Right Front RPM', 'std')]

mean_left_rear = grouped_data[('Left Rear RPM', 'mean')]
std_left_rear = grouped_data[('Left Rear RPM', 'std')]

mean_right_rear = grouped_data[('Right Rear RPM', 'mean')]
std_right_rear = grouped_data[('Right Rear RPM', 'std')]

# Create the scatter plot with error bars
plt.figure(figsize=(10, 6))

# Plot with error bars for each wheel
plt.errorbar(throttle_values, mean_left_front, yerr=std_left_front, fmt='o', label='Left Front', color='r')
plt.errorbar(throttle_values, mean_right_front, yerr=std_right_front, fmt='o', label='Right Front', color='g')
plt.errorbar(throttle_values, mean_left_rear, yerr=std_left_rear, fmt='o', label='Left Rear', color='b')
plt.errorbar(throttle_values, mean_right_rear, yerr=std_right_rear, fmt='o', label='Right Rear', color='y')

# Customize the plot
plt.title('Wheel RPM vs Throttle with Error Bars')
plt.xlabel('Throttle')
plt.ylabel('RPM')
plt.legend()
plt.grid(True)

# Save the plot as an image file
plt.savefig('wheel_rpm_vs_throttle_error_bars_plot.png')

print("Error bar plot saved as 'wheel_rpm_vs_throttle_error_bars_plot.png'")
