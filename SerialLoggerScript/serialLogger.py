import serial
import time
import serial.tools.list_ports
import keyboard

def serial_logger(port, baud_rate):
    try:
        # Initialize the serial connection
        ser = serial.Serial(port, baud_rate)
        
        # Open the file to write data
        with open('serial_data_log.txt', 'a') as f:
            print(f"Logging data from {port} at {baud_rate} baud rate. Press 'q' to stop.")
            
            while True:
                # Check if 'q' key is pressed to stop logging
                if keyboard.is_pressed('q'):
                    print("Stopping data logging...")
                    break

                # Read data from the serial port
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').strip()  # Read and decode the data
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())  # Get current timestamp
                    log_entry = f'{timestamp} - {data}'
                    
                    # Print data to the terminal
                    print(log_entry)
                    
                    # Write data to the file
                    f.write(log_entry + '\n')
                    f.flush()  # Ensure data is written to the file immediately

        ser.close()
    
    except serial.serialutil.SerialException as e:
        print(f"Error: {e}")

def main():
    # List available ports
    ports = list(serial.tools.list_ports.comports())
    print("Available ports:")
    for p in ports:
        print(p)

    # Get user input for port and baud rate
    port = input("Enter the port (e.g., COM3): ")
    baud_rate = int(input("Enter the baud rate (e.g., 115200): "))
    
    # Start the serial logger
    serial_logger(port, baud_rate)

if __name__ == "__main__":
    main()
