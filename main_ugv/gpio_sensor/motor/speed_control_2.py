from gpiozero import Robot, Motor, PWMLED, Button
import time
import threading

#motor initializations
#drone = Robot(left=Motor(4, 14), right=Motor(17, 18))
LMotor = Motor(12,13)
RMotor = Motor(26,19)
#RMotor = Motor(19,20) #<-Flipped for testing

#pwm initializations
LMPWM = PWMLED(16)
RMPWM = PWMLED(21)

#encoder initializations
LencA = Button(23)
LencB = Button(22)
RencA = Button(25)
RencB = Button(24)

#needed variables
startT = time.time()
prevT = 0

Lposi = 0
Rposi = 0
Rpos = 0
Lpos = 0
RposPrev = 0
LposPrev = 0

Rv = 0
Lv = 0
Lu = 0
LvFilt = 0

LMPH = 0
RMPH = 0
LMPHFilt = 0
RMPHFilt = 0

MPHT = 0.17

LMPHFilt = 0
LMPHPrev = 0
RMPHFilt = 0
RMPHPrev = 0
LBValue = 0
    
def readEncoderLA():
    global startT, Lposi, LBValue
    Lposi = Lposi + 1
    #if(LencB.is_pressed > 0):
    #  print("LencB.is_pressed = ", LencB.is_pressed)
    #  Lposi = Lposi + 1
    #else:
    #   Lposi = Lposi - 1

def readEncoderRA():
    global startT, Rposi
    Rposi = Rposi + 1
    #if(RencB.is_pressed > 0):
    #  Rposi = Rposi + 1
    #else:
    #   Rposi = Rposi - 1

def left_Motor():
    global Lpos, Le, Lu
    startT = time.time()
    prevT = 0

    Lpos = 0
    DTposition = 0
    Tposition = 0

    LencA.when_pressed = readEncoderLA
    LMotor.forward()
    Leintegral = 0
    LePrev = 0

    while(1):
      #read the position and velocity
      Lpos = 0
      LencA.when_pressed = None
      Lpos = Lposi
      LencA.when_pressed = readEncoderLA
      #Compute velocity with method 1
      currT = time.time() - startT
      deltaT = currT-prevT
      prevT = currT

      #Convert count/s to RPM
      pulsesPerTurn = 11*58.79 #watch out
      pulsesPerMeter = pulsesPerTurn*6.36942 #meters/second
      Target_Velocity = MPHT/2.237
      
      DTposition = Target_Velocity*pulsesPerMeter*deltaT
      Tposition = Tposition + DTposition
      #Compute the control signal 
      kp = 0.025
      ki = 0.02
      kd = 0.02
      Le = Tposition - Lpos

      Ldedt = (Le - LePrev)/(deltaT)
      Leintegral = Leintegral + (Le*deltaT)
      
      Lu = (kp*Le) + (ki*Leintegral) + (kd*Ldedt)
      if Lu < 0:
         Lu = 0
      
      Lpwr = abs(Lu)

      if Lpwr > 1:
        Lpwr = 1

      LMPWM.value = Lpwr
      LePrev = Le
      
      time.sleep(0.01)

def right_Motor():
    global MPHT, Rposi, Lposi, Le, Lu
    startT = time.time()
    prevT = 0

    Rpos = 0
    DTposition = 0
    Tposition = 0

    RencA.when_pressed = readEncoderRA
    RMotor.forward()
    Reintegral = 0
    RePrev = 0

    while(1):
      #read the position and velocity
      Rpos = 0
      RencA.when_pressed = None
      Rpos = Rposi
      RencA.when_pressed = readEncoderRA
      #Compute velocity with method 1
      currT = time.time() - startT
      deltaT = currT-prevT
      prevT = currT

      #Convert count/s to RPM
      pulsesPerTurn = 11*56
      pulsesPerMeter = pulsesPerTurn*6.36942 #meters/second
      Target_Velocity = MPHT/2.237

      DTposition = Target_Velocity*pulsesPerMeter*deltaT
      Tposition = Tposition + DTposition
      #Compute the control signal 
      kp = 0.025
      ki = 0.02
      kd = 0.02
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

      #print("Tposition = ", Tposition)
      #print("Rpos = ",Rpos)
      #print("Lpos = ",Lpos)
      #print("Re = ",Re)
      #print("Le = ", Le)
      #print("Ru = ", Ru)
      #print("Lu = ", Lu)
      #print()
      
      time.sleep(0.01)

if __name__ == "__main__":
    RM = threading.Thread(target = right_Motor)
    LM = threading.Thread(target = left_Motor)

    RM.start()
    LM.start()

    print("Motors Started.. ")