from gpiozero import Motor, PWMOutputDevice, Button
import time
from simple_pid import PID

# Initialize motor, PWM device, and encoder
motor_Ra = Motor(forward=20, backward=19)
pwm_a = PWMOutputDevice(pin=21, frequency=100)
encoder_Ra = Button(25)

motor_La = Motor(forward=13, backward=12)
pwm_b = PWMOutputDevice(pin=6, frequency=100)
encoder_La = Button(22)

# Encoder counter
counter_Ra = 0
counter_La = 0 
# Time storage
prev_time = time.time()  

# Constants for calculation
pulses_per_revolution = 616  
wheel_circumference = 15.7  

# Define function to increment encoder counter
def increment_counter_a():
    global counter_Ra
    counter_Ra += 1

def increment_counter_b():
    global counter_La
    counter_La += 1

def calculate_distance(counter, pulses_per_revolution, wheel_circumference):
    revolutions = counter / pulses_per_revolution
    distance = revolutions * wheel_circumference  # Distance in centimeters
    return distance



# Attach the increment function to the encoder's "when_pressed" event
encoder_Ra.when_pressed = increment_counter_a
encoder_La.when_pressed = increment_counter_b
#125
# PID parameters
#Kp = 1.0
#Ki = 2.0
#Kd = 5.0
#desired_revs_per_second = 1.2
desired_speed_diff = 0 #in revs per second

#PID param for motor sync 1 0.4 0.5
Kp2 = 0.01  # Increase the proportional parameter
Ki2 = 0.25  # Decrease the integral parameter
Kd2 = 0.1  # Adjust the derivative parameter as needed

# Create PID controller
#pid = PID(Kp, Ki, Kd, setpoint=desired_MPH)
#pid = PID(Kp, Ki, Kd, setpoint=desired_revs_per_second)
#pid.output_limits = (0, 0.9)  # Limit the output to PWM range (0,1)
#pid.sample_time = 0.001  # Set sample time to 0.01 seconds

#PID controller for syncing speed
pid2 = PID(Kp2, Ki2, Kd2, setpoint=desired_speed_diff)
pid2.output_limits = (0, 0.4)  # Limit the output to PWM range (0,1)
pid2.sample_time = 0.001  # Set sample time to 0.01 seconds

# Main function
def main():
    global prev_time, counter_Ra, counter_La
    
    while True:
        time.sleep(1)
        start_time = time.time() #NEW
        # Measure time elapsed since last calculation
        current_time = time.time()
        elapsed_time = current_time - prev_time
        
        # Provide an initial input
        motor_Ra.forward()
        pwm_a.value = 1.0
        motor_La.forward()
        pwm_b.value = 1.0
        
        counts_per_second_R = counter_Ra / elapsed_time
        counts_per_second_L = counter_La / elapsed_time
        revs_per_second_R = counts_per_second_R * (1/pulses_per_revolution)
        revs_per_second_L = counts_per_second_L * (1/pulses_per_revolution)



        
        # Calculate error
        error = counts_per_second_R - counts_per_second_L
        #error = desired_MPH - actual_MPH

        # Update PID controller
        pid_output = pid2(error)
        print("error:", error)

        # Adjust PWM output based on error direction
        pwm_a_correction = pid_output / 2
        pwm_b_correction = pid_output / 2

        # Ensure PWM values stay within 0 and 1 range
        if pwm_a_correction > 0:
            pwm_a.value = max(0, min(1.0 - pwm_a_correction, 1.0))
        else:
            pwm_a.value = max(0, min(1.0, 1.0 + pwm_a_correction))
        
        if pwm_b_correction > 0:
            pwm_b.value = max(0, min(1.0 - pwm_b_correction, 1.0))
        else:
            pwm_b.value = max(0, min(1.0, 1.0 + pwm_b_correction))
        
        print("Revs per second on right motor:", revs_per_second_R)
        print("Revs per second on left motor:", revs_per_second_L)
        #print("MPH: ", actual_MPH)
        print("PID Output:", pid_output)
        

        
        # If one second has passed, reset variables
        if elapsed_time >= 1.0:
            counter_Ra = 0
            counter_La = 0
            prev_time = current_time
        

if __name__ == "__main__":
    main()
