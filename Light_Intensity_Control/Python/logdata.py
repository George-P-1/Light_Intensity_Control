import serial
import csv

# Define the serial port and baud rate
serial_port = 'COM7'  # Change this to your serial port
baud_rate = 115200

# Open serial port
ser = serial.Serial(serial_port, baud_rate)

# Define list to store CON and MSR values
data_list = []

try:
    while True:
        # Read line from serial port
        line = ser.readline().decode().strip()
        
        # Check if the line starts with "REF:" and contains "CON" and "MSR"
        if line.startswith("REF:") and "CON" in line and "MSR" in line:
            # Split the line into parts based on colon ':'
            parts = line.split(":")
            
            # Extract CON and MSR values
            con_value = int(parts[5].split()[0])  # Extract CON value
            msr_value = int(parts[3].split()[0])  # Extract MSR value
            
            # Append CON and MSR values to data_list
            data_list.append([con_value, msr_value])
            
            # Print the values
            print("CON:", con_value, "MSR:", msr_value)
            
            # Save data to CSV file
            with open('data.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['CON', 'MSR'])
                for row in data_list:
                    writer.writerow(row)

except KeyboardInterrupt:
    # Close serial port on keyboard interrupt - CTRL + C
    ser.close()
    print("Serial port closed.")
