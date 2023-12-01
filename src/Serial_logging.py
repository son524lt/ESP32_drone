import serial
import time
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description='Serial Logging Script')
    parser.add_argument('--port', type=str, default='COM10', help='Serial port name')
    parser.add_argument('--baudrate', type=int, default=1000000, help='Baudrate')
    return parser.parse_args()

def main():
    args = parse_arguments()

    # Open serial port
    ser = serial.Serial(args.port, baudrate=args.baudrate, timeout=1)

    # Open a text file for logging
    log_file_path = 'serial_log.txt'
    log_file = open(log_file_path, 'w')

    try:
        while True:
            # Read data from the serial port
            try:
                data = ser.readline().decode('utf-8').strip()
            except:
                data = "?"

            # Get the current timestamp
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

            # Print the data and write it to the log file
            print(f'{timestamp} - {data}')
            log_file.write(f'{data}\n')
    except KeyboardInterrupt:
        print("Logging stopped by user.")
    finally:
        # Close the serial port and log file
        ser.close()
        log_file.close()

if __name__ == "__main__":
    main()
