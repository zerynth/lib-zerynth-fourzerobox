###############################################################################
# Modbus Serial Interface
#
# Created: 2020-09-10 15:45:48.789211
#
################################################################################

from modbus import modbus
import streams
from fourzerobox import fourzerobox
import sfw

streams.serial()

# Set Watchdog timeout
sfw.watchdog(0, 60000)

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
    print("Value", result, "successfully written in the register", register)
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
    
    config_serial = modbus.ConfigSerial(SERIAL1, 9600, rs485en=fourzerobox.RS485EN)
    print("serial configurated")
    
    # change the identifier (slave address) if needed
    master_in =  modbus.ModbusSerial(1, cfg = config_serial)
    
    print("start exchange messages")
    
    # write list of bits on register with address 2 (change it if needed)
    try:
        result = Write_One_Register(master_in, 2, values=[1, 0, 1, 0, 0, 0, 0, 0, 0, 0]) # max number of values is 16 elements  -> register 16 bit
        # read register 2 and check the result
        result = Read_One_Register(master_in, 2)
        print("Get holding register 2: ", result)
    except Exception as e:
        print(e)
        
    try:
        # write single num value on register with address 3 (change it if needed)
        result = Write_One_Register(master_in, 3, num=3)
        # read register 3 and check the result
        result = Read_One_Register(master_in, 3)
        print("Get holding register 3: ", result)
    except Exception as e:
        print(e)
    
    sfw.kick()
    
    master_in.close()
except Exception as e:
    print("Exception ", e)
    master_in.close()

