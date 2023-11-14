from gpiozero import LED
from gpiozero import PWMLED

PWM1 = PWMLED(2)
PWM2 = PWMLED(3)
en1 = LED(18)
en2 = LED(23)
m1 = LED(6)
m2 = LED(13)
m3 = LED(19)
m4 = LED(26)

while 1:
    #pwm values 0-1
    PWM1.value = 1
    PWM2.value = 1
    #enable pins on motor controller
    en1.on()
    en2.on()
    #forward on motor controller 0101
    m1.on()
    m2.off()
    m3.on()
    m4.off()