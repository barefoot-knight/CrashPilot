import time

print(dir(time))

#led = digitalio.DigitalInOut(board.D13)
#led.direction = digitalio.Direction.OUTPUT

while True:
    print(time.monotonic())
    #led.value = True
    time.sleep(0.5)
    #led.value = False
    time.sleep(0.5)
    #led.value = True
    time.sleep(0.1)
    #led.value = False
    time.sleep(0.1)