import pandas as pd
import matplotlib.pyplot as plt

# Read CSV file
data = pd.read_csv('joint_velocities.csv')  # Replace with your filename

# Create plot
plt.figure(figsize=(12, 8))
for joint in data.columns[1:]:  # Skip time column
    plt.plot(data['time'], data[joint], label=joint)

# Format plot
plt.title('Joint Velocities vs Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Joint Velocity')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Display plot
plt.save_fig('plot.png')

