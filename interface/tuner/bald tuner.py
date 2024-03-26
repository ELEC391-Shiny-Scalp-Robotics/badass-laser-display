import serial
import tkinter as tk
from tkinter import ttk
from serial.tools import list_ports
from tkinter import scrolledtext
import matplotlib.pyplot as plt

class SerialMonitorApp:
    def __init__(self, master):
        self.master = master
        master.title("BALD Tuner v6.9")

        self.serial_port = serial.Serial()
        self.serial_port.baudrate = 9600
        self.serial_port.timeout = 0.1

        self.port_label = ttk.Label(master, text="Select Port:")
        self.port_label.grid(row=0, column=0)

        self.port_combobox = ttk.Combobox(master, state="readonly")
        self.port_combobox.grid(row=0, column=1)

        self.baud_label = ttk.Label(master, text="Baud Rate:")
        self.baud_label.grid(row=0, column=2)

        self.baud_combobox = ttk.Combobox(master, values=["1000000"], state="readonly")
        self.baud_combobox.grid(row=0, column=3)
        self.baud_combobox.current(0)

        self.connect_button = tk.Button(master, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=4)

        self.disconnect_button = tk.Button(
            master, text="Disconnect", command=self.disconnect, state=tk.DISABLED
        )
        self.disconnect_button.grid(row=0, column=5)

        self.refresh_ports_button = tk.Button(
            master, text="Refresh Ports", command=self.refresh_ports
        )
        self.refresh_ports_button.grid(row=0, column=6)

        self.send_entry = ttk.Entry(master)
        self.send_entry.grid(row=2, column=0, columnspan=7, padx=5, pady=5, sticky="ew")
        self.send_entry.bind("<Return>", self.send_serial)

        self.text_area = scrolledtext.ScrolledText(
            master, width=40, height=20, state="disabled"
        )
        self.text_area.grid(row=3, column=0, columnspan=7, sticky="nsew")

        master.grid_rowconfigure(3, weight=1)
        master.grid_columnconfigure(0, weight=1)

        self.plot_data = False
        self.data_received = False
        self.data = []

    def connect(self):
        port = self.port_combobox.get()
        baudrate = int(self.baud_combobox.get())
        try:
            self.serial_port.port = port
            self.serial_port.baudrate = baudrate
            self.serial_port.open()
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            self.send_entry.config(state="normal")
            self.text_area.config(state="normal")
            self.read_serial()
        except serial.SerialException:
            print("Failed to connect to serial port.")

    def disconnect(self):
        self.serial_port.close()
        self.connect_button.config(state=tk.NORMAL)
        self.disconnect_button.config(state=tk.DISABLED)
        self.send_entry.config(state="disabled")
        self.text_area.config(state="disabled")

    def refresh_ports(self):
        available_ports = [port.device for port in list_ports.comports()]
        self.port_combobox["values"] = available_ports

    def read_serial(self):
        if self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode("utf-8")
                if line:
                    self.text_area.insert(tk.END, line)
                    self.text_area.see(tk.END)
                    if self.plot_data:
                        self.parse_data(line)
            except UnicodeDecodeError:
                print("UnicodeDecodeError: Unable to decode serial data.")
        self.master.after(3, self.read_serial)

    def send_serial(self, event):
        data = self.send_entry.get() + "\n"

        if data == "step\n":
            # clear text area
            self.text_area.config(state="normal")
            self.text_area.delete(1.0, tk.END)
            self.plot_data = True

        if self.serial_port.is_open:
            self.serial_port.write(data.encode("utf-8"))
        self.send_entry.delete(0, tk.END)

    def parse_data(self, line):
        if line.startswith("Kp:"):
            # parse Kp and Kd values if they are received
            kp, kd = map(float, line.split()[1::2])
            print("Kp:", kp, "Kd:", kd)
        elif line.startswith("ok"):
            # end of data transmission
            if self.data_received:
                self.plot_data = False
                self.data_received = False
                self.plot_graph()

        else:
            # parse data points
            data_points = list(map(int, line.strip().split()))
            self.data.append(data_points)
            self.data_received = True

    def plot_graph(self):
        data = zip(*self.data)
        time, target_x, pos_x, target_y, pos_y = data
        plt.figure()
        
        # Plot for X
        plt.subplot(2, 1, 1)
        plt.plot(time, target_x, label="Target X")
        plt.plot(time, pos_x, label="Position X")
        plt.xlabel("Time (ms)")
        plt.ylabel("Position")
        plt.title("X Axis")
        plt.legend()
        
        # Plot for Y
        plt.subplot(2, 1, 2)
        plt.plot(time, target_y, label="Target Y")
        plt.plot(time, pos_y, label="Position Y")
        plt.xlabel("Time (ms)")
        plt.ylabel("Position")
        plt.title("Y Axis")
        plt.legend()
        
        plt.tight_layout()
        plt.show()
        self.data = []

def main():
    root = tk.Tk()
    app = SerialMonitorApp(root)
    app.refresh_ports()  # Refresh available ports initially
    root.mainloop()

if __name__ == "__main__":
    main()
