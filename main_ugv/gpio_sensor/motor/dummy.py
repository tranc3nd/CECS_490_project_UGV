#from speed_control_OG import right_turn, left_turn, moving_forward, stop_and_reinitialize_motors
import speed_control_OG
import time

def main():
    print("Here")
    speed_control_OG.setup()
    
    print("WOW")
    actions = [
        ("L", 0.5, 3),  # (command, motor speed, duration in seconds)
        ("B", 0.5, 3),
        ("S", 0.5, 3)
    ]
    index = 0  # Start index
    while True:
        current_action = actions[index]
        command, speed, duration = current_action
        if command == "L":
            print("turning left")
            #speed_control_OG.right_motor("L", speed)
            speed_control_OG.left_controller("L", speed)
            speed_control_OG.right_controller("L", speed)
            #time.sleep(5)
            #stop_and_reinitialize_motors()
        elif command == "R":
            print("turning right")
            #cprint("turning right")
            #speed_control_OG.right_motor("R", speed)
            speed_control_OG.left_controller("R", speed)
            speed_control_OG.right_controller("R", speed)
            
            #time.sleep(5)
            #stop_and_reinitialize_motors()
        elif command == "B":
            print("moving foward")
            #print("Moving forward")
            speed_control_OG.left_controller("B", speed)
            speed_control_OG.right_controller("B", speed)
            #speed_control_OG.right_motor("B", speed)
        elif command == "S":
            print("Stopping motors")
            speed_control_OG.left_controller("S", speed)
            speed_control_OG.right_controller("S", speed)
            #speed_control_OG.right_motor("S", speed)
            #time.sleep(5)
            #stop_and_reinitialize_motors()
        elif command == "BW":
            print("going backwards")
            speed_control_OG.left_controller("BW", speed)
            speed_control_OG.right_controller("BW", speed)
            #speed_control_4.right_controller("B",speed)
            #speed_control_4.left_controller("B", speed)



        # Wait for the duration specified for the current action
        time.sleep(duration)
        #stop_and_reinitialize_motors()
        # Increment index to switch to the next action in the next iteration
        index = (index + 1) % len(actions)  # Cycle through the list of actions

         
if __name__ == "__main__":
    main()
