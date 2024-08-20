import tkinter as tk
import serial

values_queue = []
s = None

def serialConnect():
    global s
    portName = serialPort.get()
    baudRate = selBaudRate.get()
    baudRate = int(baudRate)
    print(f"Port:{portName} Baud Rate:{baudRate}")

    s = serial.Serial(portName,baudRate,timeout=1)
    

def serialDisconnect():
    global s  # Declare s as a global variable
    if s and s.is_open:
        s.close()
        print("Disconnected successfully")
    else:
        print("Serial port is not open or not connected")

def sendValues():
    global s
    valuetoSend = [Position.get(),
                   Kp.get(),
                   Ki.get(),
                   Kd.get()]
    floatValues = []
    toSendString = ""
    for i in range(len(valuetoSend)):
        floatValues.append(float(valuetoSend[i]))
        toSendString += "," + valuetoSend[i]
    toSendString = toSendString[1:]
    print(toSendString)
    values_queue = toSendString
    #code to send toSendString
    s.write(values_queue.encode())
    s.write(b"\n")  # Write newline character after sending values



root = tk.Tk()
root.title("Servo PID Tool")
serialPortLabel = tk.Label(root, text="SerialPort:")
serialPortLabel.grid(row=0,column=0)
serialPort = tk.Entry(root, width=10)
serialPort.grid(row=0,column=1)



selBaudRate = tk.Entry(root)
selBaudRate.grid(row=0,column=2)


connect = tk.Button(text="Connect",command=serialConnect)
connect.grid(row=0,column=3)
connect = tk.Button(text="Disconnect",command=serialDisconnect)
connect.grid(row=0,column=4)

labelRow = 1

motorNamesColumn = 0



names = ("Position","Kp","Ki","Kd")
startingRow = labelRow


PositionRow = 1
PositionLabel = tk.Label(root,text=names[0])
PositionLabel.grid(row=startingRow,column=PositionRow)
Position = tk.Entry(root,width=5)
Position.grid(row=startingRow+1, column=PositionRow, padx=5, pady=5)
Position.insert(0, "0") 


KpRow = 2
KpLabel = tk.Label(root,text=names[1])
KpLabel.grid(row=startingRow,column=KpRow)
Kp = tk.Entry(root,width=5)
Kp.grid(row=startingRow+1, column=KpRow, padx=5, pady=5)
Kp.insert(0, "0")


KiRow = 3
KiLabel = tk.Label(root,text=names[2])
KiLabel.grid(row=startingRow,column=KiRow)
Ki = tk.Entry(root,width=5)
Ki.grid(row=startingRow+1, column=KiRow, padx=5, pady=5)
Ki.insert(0, "0")



KdRow = 4
KdLabel = tk.Label(root,text=names[3])
KdLabel.grid(row=startingRow,column=KdRow)
Kd = tk.Entry(root,width=5)
Kd.grid(row=startingRow+1, column=KdRow, padx=5, pady=5)
Kd.insert(0, "0")




sendValues = tk.Button(text="Send Values",command=sendValues)
sendValues.grid(row=labelRow+5,column=1,columnspan=len(names)+1)


root.mainloop()



