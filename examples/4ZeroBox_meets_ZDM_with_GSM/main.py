###############################################################################
# 4ZeroBox meets ZDM with GSM
#
# Created: 2020-09-10 15:45:48.789211
#
###############################################################################

from bsp.drivers import gsm
import streams
from zdm import zdm
import threading as th
import sfw
import mcu
from fourzerobox import fourzerobox
import requests
import json

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
device = None

try:
    # Create FourZerobox Instance
    fzbox = fourzerobox.FourZeroBox(i2c_clk=100000)
except Exception as e:
    print(e)
    mcu.reset()

def pub_event_handler():
    global ready
    while True:
        try:
            # Sync
            ready = True
            core_sample_lock.acquire()
            ready = False
            print("======== reading")
            # Read from 4-20mA channel1, resistive channel1
            analog_val = fzbox.read_420(1)
            temperature = fzbox.read_resistive(1)
            print(" - temp:", temperature)
            print(" - analog:", analog_val)
            # Organize data in json dict
            to_send = {}
            to_send['temp'] = temperature
            to_send['analog'] = analog_val
            print("======== done")
            # Publish data to ZDM cloud service
            device.publish(to_send, "data")
            sleep(100)
            rssi = fzbox.get_rssi()
            if rssi < -70:
                fzbox.reverse_pulse('Y',100)
            else:
                fzbox.reverse_pulse('G',100)
        except Exception as e:
            print('Publish exception: ', e)
            fzbox.error_cloud()
            mcu.reset()
            
def retrieve_ts():
    if device and device.connected():
        device.mqtt.disconnect
        sleep(1000)
    print(fzbox.net.rtc())
    res = requests.get("http://now.zerynth.com/")
    js = json.loads(res.content)
    ts = int(js["now"]["epoch"])
    return ts

try:
    fzbox.net_init()
    print("attaching ...")
    fzbox.net_connect("apn")
    print("... done")
    print("Connecting to ZDM ...")
    device = zdm.Device(time_function=retrieve_ts)
    # connect the device to the ZDM
    device.connect()
    print("... done")
except Exception as e:
    print (e)
    fzbox.pulse('R', 10000)

core_sample_lock.acquire()
try:
    print("adc config...")
    # Config FourZeroBox ADC channels        
    fzbox.config_adc_010_420(1, 3, 0)
    fzbox.config_adc_resistive(1, 2, 0)
    # Config FourZeroBox ADC conversion parameters
    fzbox.set_conversion_010_420(1, 0, 100, 0, 0, 100)
    fzbox.set_conversion_resistive(1, -50, ref_table, 5, 0, None)
    print("adc config done")
except Exception as e:
    print(e)
    mcu.reset()

try:
    print("core init done")
    # Start read_event_handler thread
    thread(pub_event_handler)
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