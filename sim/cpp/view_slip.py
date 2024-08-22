import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the CSV file
data = pd.read_csv('build/slip_ratio_vs_drawbar_pull.csv')

# Plot the data
plt.figure(figsize=(10, 6))
plt.scatter(data['Slip Ratio'], data['Drawbar Pull'], marker='o')

# Add labels and title
plt.xlabel('Slip Ratio')
plt.ylabel('Drawbar Pull (N)')
plt.title('Slip Ratio vs Drawbar Pull')
plt.grid(True)
plt.ylim(-50, 50)

# Show the plot
plt.savefig('drawbarpull_vs_slip.png')

plt.show()
