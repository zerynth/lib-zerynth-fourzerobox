###############################################################################
# Modbus Serial Interface
#
# Created: 2020-09-10 15:45:48.789211
#
################################################################################

from modbus import modbus
import streams
import gpio
from fourzerobox import fourzerobox
import sfw

streams.serial()

# Set Watchdog timeout
sfw.watchdog(0, 60000)

# RS485 class with read, write, close as requested by Zerynth Modbus Library
class RS485_4zbox():
    def __init__(self, baud):
      gpio.mode(fourzerobox.RS485EN, OUTPUT)
      gpio.low(fourzerobox.RS485EN)
      self.port = streams.serial(drvname=SERIAL1, baud=baud, set_default=False)
    
    def read(self):
        bc = self.port.available()
        return self.port.read(bc)

    def write(self, packet):
        gpio.high(fourzerobox.RS485EN)
        self.port.write(packet)
        gpio.low(fourzerobox.RS485EN)

    def close(self):
        self.port.close()

def Write_One_Register(master, register, values=None, num=None):
    if (values == None and num == None):
        raise ValueError
    if (values != None):
        num = 0
        for i in range(len(values)):
            num += values[i] << i
    if (num > 0xffff):
        raise ValueError
    result = master.write_register(register, num)
    if (result == 1):
        print("Register", register, "successfully written")
    else:
        print("Register ", register, " writing failed")
    return result

def Read_One_Register(master, register):
    num = master.read_holding(register, 1)[0]
    out = []
    for i in range(10):
        out.append(num>>i & 1)
    return out
    
try:
    fzbox = fourzerobox.FourZeroBox(i2c_clk=100000)
    print("fzbox init")
    serdev = RS485_4zbox(9600)
    print("rs485 created")
    
    # change the identifier (slave address) if needed
    master_in =  modbus.ModbusSerial(1, serial_device=serdev)
    print("start exchange messages")
    
    # write list of bits on register with address 2 (chage it if needed)
    try:
        result = Write_One_Register(master_in, 2, values=[1, 0, 1, 0, 0, 0, 0, 0, 0, 0]) # max number of values is 16 elements  -> register 16 bit
        print("writing register 2 ... ")
        # read register 2 and check the result
        result = Read_One_Register(master_in, 2)
        print("Get input register 2: ", result)
    except Exception as e:
        print(e)
        
    try:
        # write single num value on register with address 3 (chage it if needed)
        result = Write_One_Register(master_in, 3, num=3)
        # read register 3 and check the result
        result = Read_One_Register(master_in, 3)
        print("Get input register 3: ", result)
    except Exception as e:
        print(e)
    
    sfw.kick()
except Exception as e:
    print("Exception ", e)
    master_in.close()