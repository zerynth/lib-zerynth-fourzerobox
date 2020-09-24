###############################################################################
# Hello 4ZeroBox
#
# Created: 2020-09-10 15:45:48.789211
#
################################################################################

# import the streams module, it is needed to send data around
import streams
import sfw

# Set Watchdog timeout
sfw.watchdog(0, 60000)

# open the default serial port, the output will be visible in the serial console
streams.serial()  

# loop forever
while True:
    print("Hello Zerynth 4ZeroBox!")   # print automatically knows where to print!
    sleep(1000)
    # Reset Watchdog timer
    sfw.kick()
