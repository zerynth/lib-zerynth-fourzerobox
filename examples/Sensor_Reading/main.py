################################################################################
# 4ZeroBox Sensor Reading
#
# Created: 2020-09-10 15:45:48.789211
#
################################################################################

import streams
import mcu
import json
import threading as th
import sfw
from fourzerobox import fourzerobox

# Lock for sync
core_sample_lock = th.Lock()
ready = True
# Set Watchdog timeout
sfw.watchdog(0, 60000)
# Init sys
streams.serial()
print('init')

# Reference table for no linear NTC sensor
ref_table = [329.5,247.7,188.5,144.1,111.3,86.43,67.77,53.41,42.47,33.90,27.28,22.05,17.96,14.69,12.09,10.00,8.313,
              6.940,5.827,4.911,4.160,3.536,3.020,2.588,2.228,1.924,1.668,1.451,1.266,1.108,0.9731,0.8572,0.7576]

# FourZero Var
fzbox = None
try:
    # Create FourZerobox Instance
    fzbox = fourzerobox.FourZeroBox(i2c_clk=100000)
except Exception as e:
    print(e)
    mcu.reset()

# Thread function for read sensor data and send them to the cloud
def read_event_handler():
    global ready

    while True:
        try:
            # Sync
            ready = True
            core_sample_lock.acquire()
            ready = False
            print("======== reading")
            # Read from 4-20mA channel1, resistive channel1, power channel1
            analog_val = fzbox.read_420(1)
            temperature = fzbox.read_resistive(1)
            power = fzbox.read_power(1)
            print(" - analog:", analog_val)
            print(" - temp:", temperature)
            print(" - power:", power)
            print("======== done")
            # reverse green blink each cycle
            fzbox.reverse_pulse('G',100)
        except Exception as e:
            print("Generic Error:", e)
            fzbox.error_cloud()
            mcu.reset()

print("core init...")
core_sample_lock.acquire()
    
try:
    print("adc config...")
    # Config FourZeroBox ADC channels        
    fzbox.config_adc_010_420(1, 3, 0)
    fzbox.config_adc_resistive(1, 2, 0)
    fzbox.config_adc_current(1, 2, 7)
    # Config FourZeroBox ADC conversion parameters
    fzbox.set_conversion_010_420(1, 0, 100, 0, 0, 100)
    fzbox.set_conversion_resistive(1, -50, ref_table, 5, 0, None)
    fzbox.set_conversion_current(1, 100, 5, 2000, 220, 0)
    print("adc config done")
except Exception as e:
    print(e)
    mcu.reset()

try:
    print("Start read_event_handler thread")
    thread(read_event_handler)
    print('start main')
    # Main Loop
    while True:
        sleep(10000)
        # Sync between main thread and pub_event_handler thread 
        if ready:
            core_sample_lock.release()
            # Reset Watchdog timer
            sfw.kick()
except Exception as e:
    print (e)
    fzbox.pulse('R', 10000)
    mcu.reset()