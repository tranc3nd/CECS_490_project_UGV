from gpiozero import Robot, Motor, PWMLED, Button
import time
import threading

#def initialization():
#global LMotor, RMotor, LMPWM, RMPWM, LencA, LencB, RencA, RencB
#motor initializations
#drone = Robot(left=Motor(4, 14), right=Motor(17, 18))
#LMotor = Motor(12,13)
#RMotor = Motor(26,19)
#RMotor = Motor(19,20) #<-Flipped for testing

#pwm initializations
LMPWM = PWMLED(16)
RMPWM = PWMLED(21)


#encoder initializations
LencA = Button(23)
LencB = Button(22)
RencA = Button(25)
RencB = Button(24)





# Define motors with their respective GPIO pins
LMotor = Motor(12,13)
RMotor = Motor(26,19)
LprevCommand = "S"
RprevCommand = "S"
MPHT = 0
direction = "S"

def stop_and_reinitialize_motors():
  # Stop motors
  LMotor.stop()
  RMotor.stop()
  cleanup_motors()
  
  # Optionally, cleanup existing motor objects if needed
def cleanup_motors():
  # Close motors
  LMotor.close()
  RMotor.close()




# Define a shared variable to signal both motors to start together NEW
#start_both_motors = False


#TIME VARIABLES
#startT = time.time()
LprevT = 0
RprevT = 0

#ENCODER RELATED 
Lposi = 0
Rposi = 0
Rpos = 0
Lpos = 0
Lu = 0




    
def readEncoderLA(): #INCREMENT COUNTER AS MOTOR TURNS (LEFT)
    global Lposi
    Lposi = Lposi + 1

def readEncoderRA(): #INCREMENT COUNTER AS MOTOR TURNS (RIGHT)
    global Rposi
    Rposi = Rposi + 1

def left_controller(navi,speed):
   global direction, MPHT
   direction = navi
   MPHT = speed

def right_controller(navi,speed):
   global direction, MPHT
   direction = navi
   MPHT = speed

def left_motor(): #include direction and MPH param after
    backward = 0
    global LprevCommand, Lposi, LprevT, direction, MPHT
    #IMPORTANT FOR FIRST ITERATION
    Lpos = 0 #LEFT MOTOR POS STORED
    DTposition = 0 #POSITION AS TIME CHANGES
    Tposition = 0 #TARGET POSITION
    #PID
    Leintegral = 0 
    LePrev = 0 
    while(1):
      LencA.when_pressed = None
      #BEGIN
      Lpos = Lposi #LEFT POSITION RAW FROM ENCODER, 
      LencA.when_pressed = readEncoderLA 

      #UPDATE CURRENT TIME AND PREV TIME
      currT = time.time() - startT 
      deltaT = currT-LprevT 
      LprevT = currT

      #Convert count/s to RPM
      pulsesPerTurn = 11*58.79 #watch out PULSES PER WHEEL TURN
      pulsesPerMeter = pulsesPerTurn*6.36942 #meters/second PULSES PER METER TRAVELED
      Target_Velocity = MPHT/2.237 #EVENTUALLY MAKE MPHT AN INPUT (MPH TO METERS PER SECOND)
      DTposition = Target_Velocity*pulsesPerMeter*deltaT #UPDATE POSITION AS TIME CHANGES, VIA ENCODER COUNT

      if (direction == "L" or direction == "S"):
        if (backward == 0): 
            LMotor.forward() #backward flag not set, continue as is
        Tposition = Lpos
        Leintegral = 0
      elif (direction == "BW"): 
        backward = 1 # set flag
        if (backward == 1):
            LMotor.backward()
            backward = 0
            if(direction!= LprevCommand):
                Lpos = Tposition
        Tposition = Tposition + DTposition

        #print("Not Moving")
      elif (direction == "B" or direction == "R"):
        if (backward == 0):
            LMotor.forward() #backward flag not set, continue as is
        if (direction != LprevCommand):
            Lpos = Tposition 
            #print(Tposition)
        Tposition = Tposition + DTposition #THE ENCODER COUNT YOU NEED TO BE ON 

      LprevCommand = direction

      #Compute the control signal PID STUFF
      kp = 0.0015
      ki = 0.0005
      kd = 0.0005

      Le = Tposition - Lpos
      Ldedt = (Le - LePrev)/(deltaT)
      Leintegral = Leintegral + (Le*deltaT)
      Lu = (kp*Le) + (ki*Leintegral) + (kd*Ldedt) #LU IS THE ADJUSTED POWER
      if Lu < 0:
         Lu = 0
      Lpwr = abs(Lu)
      if Lpwr > 1:
        Lpwr = 1
      LMPWM.value = Lpwr
      LePrev = Le
      time.sleep(0.1)
      

def right_motor():
    backward = 0
    global RprevCommand, Rposi, RprevT, direction, MPHT
    #for first iteration
    Rpos = 0
    DTposition = 0
    Tposition = 0
    #PID
    Reintegral = 0
    RePrev = 0
    while(1):
      RencA.when_pressed = None
      #BEGIN
      Rpos = Rposi
      RencA.when_pressed = readEncoderRA

      #Compute velocity with method 1
      currT = time.time() - startT
      deltaT = currT-RprevT
      RprevT = currT

      #Convert count/s to RPM
      pulsesPerTurn = 11*56
      pulsesPerMeter = pulsesPerTurn*6.36942 #meters/second
      Target_Velocity = MPHT/2.237
      DTposition = Target_Velocity*pulsesPerMeter*deltaT


      if (direction == "R" or direction == "S"):
        if (backward == 0): 
            RMotor.forward() #backward flag not set, continue as is
        Tposition = Rpos
        Reintegral = 0
      elif (direction == "BW"): 
        backward = 1 # set flag
        if (backward == 1):
            RMotor.backward()
            backward = 0
            if(direction!= RprevCommand):
                Rpos = Tposition
        Tposition = Tposition + DTposition

        #print("Not Moving")
      elif (direction == "B" or direction == "L"):
        if (backward == 0):
            RMotor.forward() #backward flag not set, continue as is
        if (direction != RprevCommand):
            Rpos = Tposition 
            #print(Tposition)
        Tposition = Tposition + DTposition #THE ENCODER COUNT YOU NEED TO BE ON 
      RprevCommand = direction

      #Compute the control signal PID STUFF
      kp = 0.0015
      ki = 0.0005
      kd = 0.0005

      Re = Tposition - Rpos
      Rdedt = (Re - RePrev)/(deltaT)
      Reintegral = Reintegral + (Re*deltaT)
      Ru = (kp*Re) + (ki*Reintegral) + (kd*Rdedt)
      if (Ru < 0):
         Ru = 0
      Rpwr = abs(Ru)
      if Rpwr > 1:
        Rpwr = 1
      RMPWM.value = Rpwr
      RePrev = Re
      time.sleep(0.1)

def setup():
  global startT, LprevCommand, RprevCommand
  LMotor.forward()
  RMotor.forward()
  LprevCommand = "S"
  RprevCommand = "S"
  startT = time.time()
  LencA.when_pressed = readEncoderLA 
  RencA.when_pressed = readEncoderRA
  LM = threading.Thread(target = left_motor, daemon=True)
  RM = threading.Thread(target = right_motor, daemon=True)
  LM.start()
  RM.start()


if __name__ == "__main__":
    #motor_param()
    pass
    #RM = threading.Thread(target = right_Motor, args = (direction, MPHT)) #debugging
    #setup()
    #LM = threading.Thread(target = left_motor, args = ("R", 0.3)) #debugging
    
    #RM = threading.Thread(target = right_motor)
    
    #BM = threading.Thread(target = moving_forward)
    

   # RM.start()
    
    #BM.start()

    #print("Motors Started.. ")