import matplotlib.pyplot as plt
import csv


with open("magnetometer_corrected.csv", 'r') as f:
    reader = csv.reader(f)
    mag_x = []
    mag_y = []
    for line in reader:
        mag_x.append(line[0])
        mag_y.append(line[1])


# Example magnetometer data (replace with your actual data)
# Replace with raw magnetometer x-axis data
# mag_x = [10, 20, 30, -10, -20, 0, -30]
# # Replace with raw magnetometer y-axis data
# mag_y = [15, 25, 35, -15, -25, 5, -35]

# Create a 2D scatter plot
plt.figure()
plt.scatter(mag_x, mag_y, c='b', label='Magnetometer Data')
plt.axhline(0, color='black', linewidth=0.5)  # Add reference line for y=0
plt.axvline(0, color='black', linewidth=0.5)  # Add reference line for x=0

# Label the axes
plt.xlabel('Magnetometer X')
plt.ylabel('Magnetometer Y')
plt.title('2D Magnetometer Data (X-Y Plane)')
plt.legend()
plt.grid()
plt.axis('equal')  # Ensure aspect ratio is 1:1 for proper analysis


print("Done Plotting")

# Show the plot
plt.show()
