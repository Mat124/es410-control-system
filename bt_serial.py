import serial

if __name__ == "__main__":
    with serial.Serial("/dev/rfcomm0", 115200) as ser:
        while True:
            rcvd = ser.readline().decode("utf-8").strip() # read a '\n' terminated line, waits for '\n'
            print(rcvd)
            ser.write("OK")