import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
import time

# Configure the serial connection (update the port as needed)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

# Initialize data lists
time_data = []
accel_x, accel_y, accel_z = [], [], []
gyro_x, gyro_y, gyro_z = [], [], []
angle_x, angle_y = [], []

# Store the starting time
start_time = time.time()

# Create the plot
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15))
fig.suptitle('MPU6050 Sensor Data')

# Initialize lines for each subplot
lines_accel = ax1.plot([], [], 'r-', [], [], 'g-', [], [], 'b-')
lines_gyro = ax2.plot([], [], 'r-', [], [], 'g-', [], [], 'b-')
lines_angle = ax3.plot([], [], 'r-', [], [], 'g-')

# Set up the axes
ax1.set_ylabel('Acceleration (g)')
ax2.set_ylabel('Gyro (degrees/s)')
ax3.set_ylabel('Angle (degrees)')
ax3.set_xlabel('Time (seconds)')

# Set up the y-axis limits
ax1.set_ylim(-2, 2)
ax2.set_ylim(-250, 250)
ax3.set_ylim(-100, 100)

# Add legends
ax1.legend(('X', 'Y', 'Z'))
ax2.legend(('X', 'Y', 'Z'))
ax3.legend(('X', 'Y'))

time_window = 20  # Show the last 20 seconds of data

def parse_serial_data(frame_data):
    """Parse a block of serial data and extract sensor values."""
    values = {}
    patterns = {
        'accel': r'Accel_X \(g\): ([-\d.]+), Accel_Y \(g\): ([-\d.]+), Accel_Z \(g\): ([-\d.]+)',
        'gyro': r'Gyro_X \(graus/s\): ([-\d.]+), Gyro_Y \(graus/s\): ([-\d.]+), Gyro_Z \(graus/s\): ([-\d.]+)',
        'angle': r'Angle_X \(graus\): ([-\d.]+), Angle_Y \(graus\): ([-\d.]+)'
    }
    
    for key, pattern in patterns.items():
        match = re.search(pattern, frame_data)
        if match:
            values[key] = [float(x) for x in match.groups()]
    
    return values

def update_plot(frame):
    """Update function for the animation."""
    frame_data = ""
    try:
        # Collect data between the start and end of a frame
        while True:
            line = ser.readline().decode('utf-8').strip()
            if '--start frame--' in line:
                frame_data = ""
            elif '--end frame--' in line:
                break
            else:
                frame_data += line + " "
        
        values = parse_serial_data(frame_data)
        
        if values:
            # Calculate the elapsed time
            elapsed_time = time.time() - start_time
            time_data.append(elapsed_time)
            
            if 'accel' in values:
                accel_x.append(values['accel'][0])
                accel_y.append(values['accel'][1])
                accel_z.append(values['accel'][2])
            
            if 'gyro' in values:
                gyro_x.append(values['gyro'][0])
                gyro_y.append(values['gyro'][1])
                gyro_z.append(values['gyro'][2])
            
            if 'angle' in values:
                angle_x.append(values['angle'][0])
                angle_y.append(values['angle'][1])
            
            # Limit data to the last 100 points
            if len(time_data) > 180:
                time_data.pop(0)
                accel_x.pop(0); accel_y.pop(0); accel_z.pop(0)
                gyro_x.pop(0); gyro_y.pop(0); gyro_z.pop(0)
                angle_x.pop(0); angle_y.pop(0)
            
            # Update the plots
            for i, line in enumerate(lines_accel):
                line.set_data(time_data, [accel_x, accel_y, accel_z][i])
            
            for i, line in enumerate(lines_gyro):
                line.set_data(time_data, [gyro_x, gyro_y, gyro_z][i])
            
            for i, line in enumerate(lines_angle):
                line.set_data(time_data, [angle_x, angle_y][i])
            
            if time_data[-1] > time_window:
                ax1.set_xlim(time_data[-1] - time_window, time_data[-1])
                ax2.set_xlim(time_data[-1] - time_window, time_data[-1])
                ax3.set_xlim(time_data[-1] - time_window, time_data[-1])
            else:
                ax1.set_xlim(0, time_window)
                ax2.set_xlim(0, time_window)
                ax3.set_xlim(0, time_window)
    
            for ax in (ax1, ax2, ax3):
                ax.relim()
                ax.autoscale_view()
    
    except KeyboardInterrupt:
        ser.close()
        plt.close()
    
    return lines_accel + lines_gyro + lines_angle

# Create the animation
anim = FuncAnimation(fig, update_plot, frames=None, interval=100, blit=True, cache_frame_data=False)

plt.tight_layout()
plt.show()

# Close the serial connection when done
ser.close()
