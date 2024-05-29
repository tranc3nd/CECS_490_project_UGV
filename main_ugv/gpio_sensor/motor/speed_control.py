from gpiozero import Motor, PWMOutputDevice, Button
import time
from simple_pid import PID

# Initialize motor, PWM device, and encoder
motor_Ra = Motor(forward=20, backward=19)
pwm_a = PWMOutputDevice(pin=21, frequency=100)
encoder_Ra = Button(25)

# Encoder counter
counter_Ra = 0
# Time storage
prev_time = time.time()  

# Constants for calculation
pulses_per_revolution = 616  
wheel_circumference = 15.7  

# Define function to increment encoder counter
def increment_counter_a():
    global counter_Ra
    counter_Ra += 1

# Attach the increment function to the encoder's "when_pressed" event
encoder_Ra.when_pressed = increment_counter_a
#125
# PID parameters
Kp = 25
Ki = 15
Kd = 5
desired_revs_per_second = 0.6
#desired_MPH = 0.4

# Create PID controller
#pid = PID(Kp, Ki, Kd, setpoint=desired_MPH)
pid = PID(Kp, Ki, Kd, setpoint=desired_revs_per_second)
pid.output_limits = (0, 1.0)  # Limit the output to PWM range (0,1)
pid.sample_time = 0.001  # Set sample time to 0.01 seconds

# Main function
def main():
    global prev_time, counter_Ra, v1
    
    while True:
        # Measure time elapsed since last calculation
        current_time = time.time()
        elapsed_time = current_time - prev_time
        
        # Provide an initial input
        motor_Ra.forward()
        pwm_a.value = 0.1
        
        counts_per_second = counter_Ra / elapsed_time
        revs_per_second = counts_per_second * (1/pulses_per_revolution)

        #actual_MPH = ((counts_per_second/616)*15.7)/44.704 #converted to mph
        
        # Calculate error
        error = desired_revs_per_second - revs_per_second
        #error = desired_MPH - actual_MPH

        # Update PID controller
        pid_output = pid(error)

        pwm_a.value = pid_output
        # Adjust PWM output based on error direction
        if revs_per_second > desired_revs_per_second:
            pwm_a.value = max(0, pwm_a.value - pid_output)
        else:
            pwm_a.value = min(1.0, pwm_a.value + pid_output)

        '''
        if actual_MPH > desired_MPH:
            pwm_a.value = max(0, pwm_a.value - pid_output)
        else:
            pwm_a.value = min(1.0, pwm_a.value + pid_output)
        '''
        
        print("Revs per second:", revs_per_second)
        #print("MPH: ", actual_MPH)
        print("PID Output:", pid_output)


        
        # If one second has passed, reset variables
        if elapsed_time >= 1.0:
            counter_Ra = 0
            prev_time = current_time

if __name__ == "__main__":
    main()
