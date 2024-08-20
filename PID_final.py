from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
import keyboard
import krpc
import math
import time

time.sleep(2)

# User Settings
draw_graph = True            #Choose if the graph will be created or not
touchDownSpeed = 3.0         #Choose your desired touch down speed
speed_scale = 8.5           #Value that multiplies square root
a = 0.12                     #Angular coeficient of the line
rocket_height = 13.34        #height of the rocket itself
           
#Advice:
# To get the height of your rocket, run the program with the rocket landed, the correct rocket_height
# value will be printed in terminal. If you are in launch pad, 
# subtract 4.15 to get real rocket_height. 
  
#Setup
conn = krpc.connect(name='Automated Landing') #Using default server 127.0.0.1
vessel = conn.space_center.active_vessel
control = vessel.control    
reference_frame = vessel.orbit.body.reference_frame      
b = touchDownSpeed - (a*rocket_height)
intersect_altitude = [0.0, 0.0]
x = sp.symbols('x')
control_input = 0.0
desired_speed = 0.0
dt = 0.01  # PID Time step

# Flight data
reference_frame = vessel.surface_velocity_reference_frame
flight_info = vessel.flight(vessel.orbit.body.reference_frame)
current_speed = flight_info.speed
altitude = flight_info.surface_altitude
eq = (speed_scale * sp.sqrt(x)) - (a * x + b)
intersect_altitude = sp.solve(eq, x)
plt.switch_backend('TkAgg')
altitude_data = []
velocity_data = []
speed_error = 0.0

#if we get two intersect_altitude, choose the higher
if(intersect_altitude[0] > 100):
    intersect_altitude = intersect_altitude[0]
else:
    intersect_altitude = intersect_altitude[1]
print(f'Intersect Altitude: {intersect_altitude} meters')

#Rocket preparation
control.sas = True
time.sleep(0.2)
control.rcs = True
time.sleep(0.2)
keyboard.press('b')
time.sleep(0.2)
keyboard.release('b')
if flight_info.speed > 1:
    control.sas_mode = conn.space_center.SASMode.retrograde

if(draw_graph):
    
    # Graph settings
    plt.ion()  
    fig, ax = plt.subplots()
    line, = ax.plot([], [], label='Altitude vs Velocity')
    reference_line, = ax.plot([], [], label=f'Desired Velocity ({speed_scale}*sqrt(altitude))', linestyle='--')
    custom_line, = ax.plot([], [], label=f'Velocity = {a}*Altitude + ({b})', linestyle=':')

    ax.set_xlabel('Altitude (m)')       
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Altitude vs Velocity')
    ax.legend()
    ax.grid(True)

def update_plot():
    line.set_xdata(altitude_data)
    line.set_ydata(velocity_data)
    
    altitude = flight_info.surface_altitude
    
    # Update reference square root
    if(altitude >= intersect_altitude):
        desired_velocity_data = [speed_scale * np.sqrt(alt) for alt in altitude_data]
        reference_line.set_xdata(altitude_data)
        reference_line.set_ydata(desired_velocity_data)
    
    # Update line
    if altitude_data:  
        altitudes_custom = [0, intersect_altitude]
        velocities_custom = [0, a * intersect_altitude + b]
        custom_line.set_xdata(altitudes_custom)
        custom_line.set_ydata(velocities_custom)
    
    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.01)

class PID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# Set PID constants
pid = PID(P = 0.0742    , I = 0.00903, D = 0.000421)

print("Start")
try:
    #Main loop
    while True:
        altitude = flight_info.surface_altitude
        velocity = vessel.flight(vessel.orbit.body.reference_frame).speed
            
        altitude_data.append(altitude)
        velocity_data.append(velocity)
    
        if(draw_graph):
            update_plot()
        
        current_speed = flight_info.speed
        altitude = flight_info.surface_altitude

        # Calculate desired speed
        if altitude >= intersect_altitude:
            desired_speed = speed_scale*math.sqrt(altitude)
        else:
            desired_speed = a*altitude + b
        
        # Deploy landing gear
        if 30.0 < current_speed < 40.0:
            keyboard.press('g')

        # Switch SAS mode to radial 
        if 10.0 < current_speed < 15.0:  
            control.sas_mode = conn.space_center.SASMode.radial
            
        speed_error = current_speed - desired_speed
        
        #Calculate control input
        if(speed_error > -100):
            control_input = pid.update(speed_error, dt)
            vessel.control.throttle = control_input
            time.sleep(dt)

        # Break the loop if the speed is sufficiently low
        if current_speed < 0.4:
            vessel.control.throttle = 0.0
            break

        print("Speed error: ", speed_error)

except KeyboardInterrupt:
    print("Program interrupted by user")
    
vessel.control.throttle = 0.0
keyboard.press('b')
time.sleep(1)
keyboard.release('b')
control.rcs = False
time.sleep(1)
control.sas = False
time.sleep(1)
altitude = flight_info.surface_altitude
print("\nrocket_height = ", altitude)

print("\nLanding completed\n")

# Keep graph open after the program end
plt.ioff()
plt.show()