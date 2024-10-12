from connect import scan_for_drones, connect_to_drone, arm_drone, disarm_drone
import connect as con
import tkinter as tk
def create_button(master, text, row, col, colspan=1,target=None):
    button = tk.Button(master, text=text,command=target)
    button.grid(row=row, column=col, columnspan=colspan, sticky="nsew", padx=5, pady=5)
    return button
def setup_connection():
    telemetry_port =input_entry.get()
    connection = connect_to_drone(telemetry_port)
    drones = scan_for_drones(connection)
    for drone in drones:
        if drone==2:
            DronePanel(f"sys_id:{drone}", len(input_list), isLeader=True, connection=connection)
        else:
            DronePanel(f"sys_id:{drone}", len(input_list), connection=connection)  

def toggle_arm(self):
    if self.arm_state.get():
        self.arm_button.config(text="DISARM")
        arm_drone(self.connection, self.vehicle_id)
        print("Drone armed")
    else:
        self.arm_button.config(text="ARM")
        disarm_drone(self.connection, self.vehicle_id)
        print("Drone disarmed")
    self.arm_state.set(not self.arm_state.get())
class DronePanel:
    
    def __init__(self,label_text,vehicle_id,isLeader=False,connection=None):
        self.connection=connection
        self.Battery=con.get_battery(self.connection),
        self.Altitude=con.get_altitude(self.connection),
                            # "Throttle":get_throttle(self),
        self.Relative_Altitude=con.get_rel_altitude(self.connection),
        self.Status=con.get_status(self.connection),
        self.Prearm=con.get_prearm(self.connection),
                            # "Offset":get_offset(self)
                            
        self.frame = tk.Frame(main_frame)
        self.frame.pack(side="left")
        self.vehicle_id = vehicle_id
        self.label_text = label_text
        self.status_label = None
        self.arm_state = tk.BooleanVar(value=True)
        self.prearm_label = None
        self.arm_button = None
        self.guided_t10_button = None
        self.mode_guided_button = None
        self.mode_rtl_button = None
        self.kill_button = None
        self.mode_follow_button = None
        self.isLeader=isLeader
        if self.isLeader:
            self.create_leader_panel()
        else:
            self.create_drone_panel()
    
    def create_leader_panel(self):
        # Create a Label for the vehicle header
        label = tk.Label(self.frame, text=self.label_text, fg="red", font=("Arial", 10, "bold"))
        label.grid(row=0, column=0, columnspan=3, sticky="w", padx=5, pady=5)

        # Label displaying prearm status
        prearm_label = tk.Label(self.frame, text=f"Battery : {self.Battery}\nAlt : {self.Altitude}\nRel Alt : {self.Relative_Altitude}\nStatus : {self.Status}\nPrearm : {self.Prearm}", justify="left")
        prearm_label.grid(row=1, column=0, columnspan=3, sticky="w", padx=5, pady=5)

        # Creating buttons
        self.arm_button=create_button(self.frame, "ARM", 2, 0,target=lambda: toggle_arm(self))
        create_button(self.frame, "ALL", 2, 1)

        create_button(self.frame, "GUIDED T/O 10m", 3, 0)
        create_button(self.frame, "ALL", 3, 1)

        create_button(self.frame, "Mode GUIDED", 4, 0)
        create_button(self.frame, "ALL", 4, 1)

        create_button(self.frame, "Mode RTL", 5, 0)
        create_button(self.frame, "ALL", 5, 1)

        create_button(self.frame, "KILL", 6, 0)
        create_button(self.frame, "ALL", 6, 1)

        create_button(self.frame, "All Follow Leader", 7, 0, colspan=2)

        create_button(self.frame, "Mode AUTO", 8, 0, colspan=2)
    
    def create_drone_panel(self):
        # Create a Label for the vehicle header
        label = tk.Label(self.frame, text=self.label_text, fg="red", font=("Arial", 10, "bold"))
        label.pack()

        # Status Labels (for Alt, Thr, Battery, etc.)
        self.status_label = tk.Label(self.frame, text="Alt: 597m    Thr: 0% \nRel Alt: 0m\nBattery: 25.0V\nStatus: On Ground\nPrearm: OK\nOffset: 0m, 0m")
        self.status_label.pack()

        # Prearm information
        self.prearm_label = tk.Label(self.frame, text="PreArm: GPS waiting for home\nPreArm: Fence requires position")
        self.prearm_label.pack()

        # Buttons for control options
        self.arm_button = tk.Button(self.frame, text="ARM",command=lambda: print(f"ARMed Vehicle {self.label_text}:{self.vehicle_id}"))
        self.arm_button.pack(fill="x")

        self.guided_t10_button = tk.Button(self.frame, text="GUIDED T/O 10m", command=lambda: print("GUIDED T/O 10m"))
        self.guided_t10_button.pack(fill="x")

        self.mode_guided_button = tk.Button(self.frame, text="Mode GUIDED"    )
        self.mode_guided_button.pack(fill="x")

        self.mode_rtl_button = tk.Button(self.frame, text="Mode RTL")
        self.mode_rtl_button.pack(fill="x")

        self.kill_button = tk.Button(self.frame, text="KILL")
        self.kill_button.pack(fill="x")

        self.mode_follow_button = tk.Button(self.frame, text="Mode Follow")
        self.mode_follow_button.pack(fill="x")

root = tk.Tk()
root.title("MAVProxy: Swarm Control")
main_frame = tk.Frame(root)
main_frame.pack(padx=10, pady=10)
input_list = []
input_frame = tk.Frame(main_frame)
input_frame.pack(pady=10)
input_label = tk.Label(input_frame, text="Enter value:")
input_label.pack(side="left")
root.bind("<Return>", lambda event: [setup_connection()])
input_entry = tk.Entry(input_frame)
input_entry.pack(side="left")
listbox_frame = tk.Frame(main_frame)
listbox = tk.Listbox(listbox_frame)
# swarmUI({})
# VehiclePanel("copter",1,1,1,1,1,1,10)
root.mainloop() 
