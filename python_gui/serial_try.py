import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib as mpl
import serial
import serial.tools.list_ports
import threading
import time

# === Globals ===
ser = None
polling_interval = 0.1  # seconds
start_time = time.time()
x_data, angle_x, angle_y, angle_z = [], [], [], []
y_max_value = 180
theme = "#444242"

# === Plot Styling ===
mpl.rcParams.update({
    "axes.facecolor": theme,
    "figure.facecolor": theme,
    "axes.edgecolor": "white",
    "axes.labelcolor": "white",
    "xtick.color": "white",
    "ytick.color": "white",
    "text.color": "white",
    "legend.facecolor": theme,
    "legend.edgecolor": "white"
})

# === GUI ===
root = tk.Tk()
root.title("Servo + Angle Plotter")
root.geometry("1000x900")
root.configure(bg=theme)

style = ttk.Style()
style.theme_use("default")
style.configure("TLabel", background=theme, foreground="white")
style.configure("TButton", background="#333333", foreground="white")
style.configure("TScale", background=theme, troughcolor=theme)

# === Serial Connection UI ===
connect_frame = tk.Frame(root, bg=theme)
connect_frame.pack(pady=10)

port_label = ttk.Label(connect_frame, text="COM Port:")
port_label.pack(side=tk.LEFT, padx=5)

port_var = tk.StringVar()
ports = [port.device for port in serial.tools.list_ports.comports()]
port_dropdown = ttk.Combobox(connect_frame, textvariable=port_var, values=ports, state='readonly', width=15)
port_dropdown.pack(side=tk.LEFT)

def connect_serial():
    global ser
    try:
        ser = serial.Serial(port_var.get(), 9600, timeout=1)
        time.sleep(2)
        status_label.config(text=f"Connected to {port_var.get()}")
        threading.Thread(target=read_serial, daemon=True).start()
        threading.Thread(target=poll_info, daemon=True).start()
    except Exception as e:
        status_label.config(text=f"Failed: {e}")

connect_button = ttk.Button(connect_frame, text="Connect", command=connect_serial)
connect_button.pack(side=tk.LEFT, padx=10)

status_label = ttk.Label(connect_frame, text="Not connected")
status_label.pack(side=tk.LEFT, padx=10)

# === Servo Slider ===
control_frame = tk.Frame(root, bg=theme)
control_frame.pack(pady=10)

value_label = ttk.Label(control_frame, text="Value: 90")
value_label.grid(row=0, column=0, padx=10)

def send_slider_value(val):
    value = int(float(val))
    if ser and ser.is_open:
        cmd = f"{value}\n"
        ser.write(cmd.encode())
        value_label.config(text=f"Value: {value}")
        sent_text.insert(tk.END, cmd)
        sent_text.see(tk.END)

def stop_slider():
    slider.set(90)
    send_slider_value("90")

slider = ttk.Scale(control_frame, from_=0, to=180, orient='horizontal', command=send_slider_value, length=400)
slider.set(90)
slider.grid(row=0, column=1, padx=10)

stop_button = ttk.Button(control_frame, text="Reset to 90", command=stop_slider)
stop_button.grid(row=0, column=2, padx=10)

# === Custom Command ===
command_frame = tk.Frame(root, bg=theme)
command_frame.pack(pady=10)

command_label = ttk.Label(command_frame, text="Custom Command (e.g., :ROT|22.5|27.8|90|:):")
command_label.pack(side=tk.LEFT, padx=5)

command_entry = ttk.Entry(command_frame, width=40)
command_entry.pack(side=tk.LEFT, padx=5)

def send_command():
    cmd = command_entry.get().strip()
    if ser and ser.is_open and cmd:
        if not cmd.endswith("\n"):
            cmd += "\n"
        ser.write(cmd.encode())
        sent_text.insert(tk.END, cmd)
        sent_text.see(tk.END)

send_button = ttk.Button(command_frame, text="Send", command=send_command)
send_button.pack(side=tk.LEFT, padx=5)

# === Arrow Key Bindings ===
def increase_slider(event=None):
    slider.set(min(slider.get() + 1, 180))
    send_slider_value(slider.get())

def decrease_slider(event=None):
    slider.set(max(slider.get() - 1, 0))
    send_slider_value(slider.get())

root.bind("<Right>", increase_slider)
root.bind("<Left>", decrease_slider)
root.bind("<Up>", increase_slider)
root.bind("<Down>", decrease_slider)

# === Plot Setup ===
fig, ax = plt.subplots(figsize=(8, 4))
fig.patch.set_facecolor(theme)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

line1, = ax.plot([], [], label="Angle X", color='cyan')
line2, = ax.plot([], [], label="Angle Y", color='magenta')
line3, = ax.plot([], [], label="Angle Z", color='yellow')

ax.set_ylabel("Angle (°)")
ax.set_xlabel("Time (s)")
ax.set_xlim(0, 10)
ax.set_ylim(-y_max_value, y_max_value)
ax.legend()
ax.grid(True)

def update_y_axis(val):
    global y_max_value
    y_max_value = int(float(val))
    ax.set_ylim(-y_max_value, y_max_value)
    canvas.draw()

# === Y-Axis Slider ===
y_control_frame = tk.Frame(root, bg=theme)
y_control_frame.pack(pady=10)

y_label = ttk.Label(y_control_frame, text="Max Y-Axis: 180")
y_label.pack(side=tk.LEFT, padx=10)

y_slider = ttk.Scale(y_control_frame, from_=2, to=360, orient='vertical', length=300,
                     command=lambda val: [update_y_axis(val), y_label.config(text=f"Max Y-Axis: {int(float(val))}")])
y_slider.set(180)
y_slider.pack(side=tk.LEFT)

# === Serial Polling ===
def poll_info():
    while ser and ser.is_open:
        try:
            ser.write(b":|GETINFO|:\n")
            sent_text.insert(tk.END, ":|GETINFO|:\n")
            sent_text.see(tk.END)
        except:
            pass
        time.sleep(polling_interval)

# === Plot Updating ===
def update_plot():
    line1.set_data(x_data, angle_x)
    line2.set_data(x_data, angle_y)
    line3.set_data(x_data, angle_z)
    if x_data:
        ax.set_xlim(max(0, x_data[-1] - 10), x_data[-1] + 1)
    ax.set_ylim(-y_max_value, y_max_value)
    canvas.draw()

# === Serial Reading Thread ===
def read_serial():
    while ser and ser.is_open:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                received_text.insert(tk.END, line + "\n")
                received_text.see(tk.END)
            if line.startswith(":INFO|") and line.endswith("|:"):
                parts = line[6:-2].split("|")
                if len(parts) == 3:
                    t = time.time() - start_time
                    x, y, z = map(float, parts)
                    x_data.append(t)
                    angle_x.append(x)
                    angle_y.append(y)
                    angle_z.append(z)
                    if len(x_data) > 200:
                        x_data.pop(0)
                        angle_x.pop(0)
                        angle_y.pop(0)
                        angle_z.pop(0)
                    update_plot()
        except Exception as e:
            print("Read Error:", e)

# === Logs ===
log_frame = tk.Frame(root, bg=theme)
log_frame.pack(pady=10)

sent_label = ttk.Label(log_frame, text="Sent:")
sent_label.grid(row=0, column=0, sticky='w')
sent_text = tk.Text(log_frame, height=10, width=50, bg="#222222", fg="cyan", insertbackground="white")
sent_text.grid(row=1, column=0, padx=5, pady=5)

received_label = ttk.Label(log_frame, text="Received:")
received_label.grid(row=0, column=1, sticky='w')
received_text = tk.Text(log_frame, height=10, width=50, bg="#222222", fg="magenta", insertbackground="white")
received_text.grid(row=1, column=1, padx=5, pady=5)

# === Start GUI ===
root.mainloop()
