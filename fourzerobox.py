"""
.. module:: fourzerobox_lib

***************
4ZeroBox Module
***************

This module contains the driver for enabling and handling all 4ZeroBox onboard features.
The 4ZeroBox class permits an easier access to the internal peripherals and exposes all functionalities in simple function calls.

.. warning:: this library is using the Zerynth *conditional compilation* feature. This means that some of the methods documented below must be
explicitly enabled in a separate configuration file called `project.yml` (or using the Zerynth Studio configuration browser).

Here Below a list of configurable defines for this library:


   Example: ::
   
        # project.yml
        config:
            # enable ethernet connection
            NETWORK_ETH: null
            # enable wifi connection
            NETWORK_WIFI: true
            # enable gsm connection
            NETWORK_GSM: true
            # enable ADC 0-10v/4-20mA peripheral
            ADC_010_420: null
            # enable ADC resistive peripheral
            ADC_RESISTIVE: null
            # enable ADC current peripheral
            ADC_CURRENT: null
            # enable can peripheral
            CAN_ENABLE: null
            # enable RS485 peripheral
            RS485_ENABLE: null
            # enable RS232 peripheral
            RS232_ENABLE: null
            # enable SD Card
            SDCARD_ENABLED: null
            # enable DEBUG for fourzerobox
            DEBUG_FZB: null

    """

import gpio
import threading
from semtech.sx1503 import sx1503
from texas.ads1015 import ads1015
#-if SDCARD_ENABLED
import fatfs
#-endif
import flash
import json
net_driver = None
net = None
#-if NETWORK_WIFI
from bsp.drivers import wifi as net
#-endif
#-if NETWORK_ETH
from bsp.drivers import eth as net
#-endif
#-if NETWORK_GSM
from bsp.drivers import gsm as net
#-endif
import streams
#-if CAN_ENABLED
from microchip.mcp2515 import mcp2515
#-endif
#-if RS232_ENABLED
import timers
#-endif
#-if RS485_ENABLED
import timers
#-endif

new_exception(NetworkInitializedException, Exception)
new_exception(NetworkConnectionException, Exception)
new_exception(NetworkInitException, Exception)

BAUD_232 = 115200
BAUD_485 = 115200
SD_FREQ = 20000

# CAN MODES
MCP_STDEXT  = 0
MCP_STD     = 1
MCP_EXT     = 2
MCP_ANY     = 3

# ACD range|pga =   0,     1,     2,     3,     4,     5,     6,     7
VREF_ADS1015 = [6.144, 4.096, 2.048, 1.024, 0.512, 0.256, 0.256, 0.256]

RS485EN  = D40
AN_2     = D41
IO_2     = D42
PWM_2    = D43
INT_2    = D44
REL_1    = D45
REL_2    = D46
LED_R    = D47
ISO_2    = D48
SNK_1    = D49
SNK_2    = D50
#-if FZB_V7
BATT     = D51
#-else
nRST     = D51
#-endif
EXTV     = D52
ISO_1    = D53
LED_G    = D54
LED_B    = D55

SPI_CLICK1 = SPI0
SPI_CLICK2 = SPI0
SERIAL_CLICK1 = SERIAL2
SERIAL_CLICK2 = SERIAL2
I2C_CLICK1 = I2C0
I2C_CLICK2 = I2C0
CS_CLICK1 = D33
CS_CLICK2 = D33
RST_CLICK1 = D42
RST_CLICK2 = D42
AN_CLICK1 = D34
AN_CLICK2 = D41
INT_CLICK1 = D39
INT_CLICK2 = D44
PWM_CLICK1 = D16
PWM_CLICK2 = D43

LEDS = {
    'R' : (LED_R,),
    'G' : (LED_G,),
    'B' : (LED_B,),
    'M' : (LED_R, LED_B),
    'Y' : (LED_R, LED_G),
    'C' : (LED_G, LED_B),
    'W' : (LED_R, LED_G, LED_B),
}
## LABELS
PGA = 0
SPS = 1
VREF = 2
Y_MIN = 3
Y_MAX = 4
OFFSET = 5
UNDER_X = 6
OVER_X = 7
V_MIN = 8
REF_TABLE = 9
DELTA = 10
OUT_OF_RANGE = 11
N_SAMPLES = 12
NCOIL = 13
RATIO = 14
VOLTAGE = 15



class FourZeroBox():
    def __init__(self, i2c_clk=400000, spi_clk=1000000):
        '''

.. class:: FourZeroBox(i2c_clk=400000, spi_clk=1000000)

        Initialize the class specifying what features must be enabled and
        configuring the clock speed of I2C and SPI protocols.

        :param i2c_clk:  Clock for connected I2C devices, in Hz. (D=400000)
        :param spi_clk:  Clock for connected SPI devices, in Hz. (D=1000000)

        '''
        if i2c_clk not in (100000, 400000, 1000000):
            raise ValueError
        if spi_clk not in (800000, 1000000, 8000000, 10000000):
            raise ValueError

        #-if DEBUG_FZB
        self.FZprint("set vars")
        #-endif

        self._spi_en = False
        self._sd_en = False
        self.i2c_clk = i2c_clk
        self.spi_clk = spi_clk

        #-if DEBUG_FZB
        self.FZprint("init lock")
        #-endif

        # lock init
        self.lockI2C = threading.Lock() # lock for I2C0
        self.lockSPI = threading.Lock() # lock for SPI0
        # self.lockSERIAL1 = threading.Lock() # lock for SERIAL1
        self.lockFLASH = threading.Lock() # lock for internal flash

        #-if DEBUG_FZB
        self.FZprint("init pe")
        #-endif

        self._init_port_expander()

        #-if ADC_010_420
        self._adc_config = []
        self._adc_010_conversion = []
        self._adc_420_conversion = []
        self._init_adc_010_420()
        ##-if DEBUG_FZB
        self.FZprint("adc 010_420 enabled")
        ##-endif
        #-endif

        #-if ADC_RESISTIVE
        self._adc_resistive_config = []
        self._adc_resistive_conversion = []
        self._init_adc_resistive()
        ##-if DEBUG_FZB
        self.FZprint("adc resistive enabled")
        ##-endif
        #-endif

        #-if ADC_CURRENT
        self._adc_current_config = []
        self._adc_current_conversion = []
        self._init_adc_current()
        ##-if DEBUG_FZB
        self.FZprint("adc current enabled")
        ##-endif
        #-endif

        self._eth_en = False
        self._wifi_en = False
        self._gsm_en = False

        #-if CAN_ENABLED
        self._spi_en = True
        self._init_can()
        ##-if DEBUG_FZB
        self.FZprint("can enabled")
        ##-endif
        #-endif

        #-if RS485_ENABLED
        self._init_rs485()
        ##-if DEBUG_FZB
        self.FZprint("rs485 enabled")
        ##-endif
        #-endif

        #-if RS232_ENABLED
        self._init_rs232()
        ##-if DEBUG_FZB
        self.FZprint("rs232 enabled")
        ##-endif
        #-endif

        #-if DEBUG_FZB
        self.FZprint("to enable sd card feat call sd_init method")
        #-endif

        # ethernet/wifi/gsm initialized
        self.net_initialized = False

        self.net = False
        self.net_driver = False
        self.check_net = []
        #-if NETWORK_WIFI
        self.check_net.append("wifi")
        #-endif
        #-if NETWORK_ETH
        self.check_net.append("eth")
        #-endif
        #-if NETWORK_GSM
        self.check_net.append("gsm")
        ##-if DEBUG_FZB
        self.FZprint("to enable gsm modem remember to turn on the related switches")
        ##-endif
        #-endif
        if len(self.check_net) > 1:
            raise UnsupportedError
        elif len(self.check_net) == 1:
            self.net = net.interface()
            self.net_driver = net

    def _init_port_expander(self):
        # setting pinmap
        pinmap = {                      # now add pins definition for the expander
            LED_R   : 7,                       # LED R mapped to pin 7 on sx1503
            LED_G   : 14,                      # LED G mapped to pin 14 on sx1503
            LED_B   : 15,                      # LED B mapped to pin 15 on sx1503
            REL_1   : 5,                       # RELAY1 mapped to pin 5 on sx1503
            REL_2   : 6,                       # RELAY2 mapped to pin 6 on sx1503
            ISO_1   : 13,                      # OPTO1 mapped to pin 13 on sx1503
            ISO_2   : 8,                       # OPTO2 mapped to pin 8 on sx1503
            EXTV    : 12,                      # EXTERNAL_VOLTAGE_DETECT mapped to pin 12 on sx1503
            #-if FZB_V7
            BATT    : 11,                      # BATTERY_STATUS mapped to pin 11 on sx1503
            #-else
            nRST    : 11,                      # RESET_ETH mapped to pin 11 on sx1503
            #-endif                      
            SNK_1   : 9,                       # SINK1 mapped to pin 9 on sx1503
            SNK_2   : 10,                      # SINK2 mapped to pin 10 on sx1503
            PWM_2   : 3,                       # PWM pin on MIKROBUS slot 2 mapped to pin 3 on sx1503
            INT_2   : 4,                       # INT pin on MIKROBUS slot 2 mapped to pin 4 on sx1503
            AN_2    : 1,                       # AN pin on MIKROBUS slot 2 mapped to pin 1 on sx1503
            IO_2    : 2,                       # PEIO2 on SCREW connector mapped to pin 2 on sx1503
            RS485EN : 0,                       # RS485 pin trasmit/receive
        }

        # initialize port expander driver
        port_expander = sx1503.SX1503(I2C0, clk=self.i2c_clk)
        # add port expander
        gpio.add_expander(1, port_expander, pinmap)
        # init gpio components

        # we set pins to low before setting the mode to avoid click noise at
        # startup of the board
        gpio.low(SNK_1)
        gpio.low(SNK_2)
        gpio.low(REL_1)
        gpio.low(REL_2)

        gpio.mode(LED_R,OUTPUT)
        gpio.mode(LED_G,OUTPUT)
        gpio.mode(LED_B,OUTPUT)
        gpio.mode(REL_1,OUTPUT)
        gpio.mode(REL_2,OUTPUT)
        gpio.mode(ISO_1,INPUT)
        gpio.mode(ISO_2,INPUT)
        gpio.mode(SNK_1,OUTPUT)
        gpio.mode(SNK_2,OUTPUT)
        gpio.mode(EXTV,INPUT)
        #-if FZB_V7
        gpio.mode(BATT,INPUT)
        #-else
        gpio.mode(nRST, OUTPUT)
        gpio.high(nRST)
        #-endif        
        gpio.mode(RS485EN,OUTPUT)
        gpio.mode(RS485EN,OUTPUT)

        gpio.low(RS485EN)
        # already high by default
        gpio.high(LED_R)
        gpio.high(LED_G)
        gpio.high(LED_B)

    def _check_adc_config(self, pga, sps):
        if pga not in (0,1,2,3,4,5,6,7):
            raise ValueError
        elif sps not in (0,1,2,3,4,5,6,7):
            raise ValueError


#-if ADC_010_420
    def _init_adc_010_420(self):
        # 0-10V / 4-20mA adc
        self.lockI2C.acquire()
        self.adc = ads1015.ADS1015(I2C0, addr=73, clk=self.i2c_clk)
        self._adc_config = [
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]},
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]},
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]},
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]}
        ]
        self._adc_conversion = [
            {Y_MIN: 0, Y_MAX: 100, OFFSET: 0, UNDER_X: 0, OVER_X: 100},
            {Y_MIN: 0, Y_MAX: 100, OFFSET: 0, UNDER_X: 0, OVER_X: 100},
            {Y_MIN: 0, Y_MAX: 100, OFFSET: 0, UNDER_X: 0, OVER_X: 100},
            {Y_MIN: 0, Y_MAX: 100, OFFSET: 0, UNDER_X: 0, OVER_X: 100}
        ]
        self.lockI2C.release()


    def config_adc_010_420(self, ch, pga, sps):
        '''

.. method:: config_adc_010_420(ch, pga, sps)

        This method configure one of the four 0-10V/4-20mA ADC channels
        (chosen using the `ch` parameter).

        :param ch: Chose the channel to be configured (can be 1, 2, 3, 4).
        :param pga: Set the PGA Gain.
        :param sps: Set the samples-per-second data rate.
        
        '''
        # config adc 010_420 channels
        if ch not in (1,2,3,4):
            raise ValueError
        self._check_adc_config(pga, sps)
        idx = ch - 1
        self._adc_config[idx][PGA]  = pga
        self._adc_config[idx][SPS]  = sps
        self._adc_config[idx][VREF] = VREF_ADS1015[pga]


    # set conversion parameters
    def set_conversion_010_420(self, ch, y_min, y_max, offset=0, under_x=None, over_x=None):
        '''

.. method:: set_conversion_010_420(ch, y_min, y_max, offset=0, under_x=None, over_x=None)

        Set the conversion method to be applied for the data read from the
        0-10V/4-20mA ADC. The conversion scale can be configured using the
        `y_min` and `y_max` parameters, and optionally it can also be set
        an `offset` and tresholds for the input data.

        :param ch: Chose the channel to be configured (can be 1, 2, 3, 4).
        :param y_min: Set the minimum value for the linear scale.
        :param y_max: Set the maximum value for the linear scale.
        :param offset: Optionally set an offset to be applied. (D=0)
        :param under_x: Optionally set a minimum treshold. (D=None)
        :param over_x: Optionally set a maximum treshold. (D=None)

        '''
        # if under_x is None: under_x = y_min
        # if over_x is None: over_x = y_max
        if ch not in (1,2,3,4):
            raise ValueError
        idx = ch - 1
        self._adc_conversion[idx][Y_MIN]   = y_min
        self._adc_conversion[idx][Y_MAX]   = y_max
        self._adc_conversion[idx][OFFSET]  = offset
        self._adc_conversion[idx][UNDER_X] = under_x
        self._adc_conversion[idx][OVER_X]  = over_x


    def read_010(self, ch, raw=False, electric_value=False):
        '''

.. method:: read_010(ch, raw=False, electric_value=False)

        Read value from the 0-10V ADC. It is possible to get the raw
        data from the ADC, or the electric value of the read signal.

        :param ch: Select the ADC channel to read from. Can be one of 1, 2, 3, 4.
        :param raw: If set, the raw data of the ADC is returned. (D=False)
        :param electric_value: If set, the electric value is returned. (D=False)
        
        '''
        if raw and electric_value:
            raise UnsupportedError
        if ch not in (1,2,3,4):
            raise ValueError
        idx = ch - 1
        channel = ch + 3
        self.lockI2C.acquire()
        self.adc.set(os=0, ch=channel, pga=self._adc_config[idx][PGA], sps=self._adc_config[idx][SPS])
        data = self.adc.get_raw_data()
        self.lockI2C.release()
        if raw:
            return data
        edata = self._convert_010(data, self._adc_config[idx][VREF])
        if electric_value:
            return edata
        return self._scale_010_420(edata, 0, 10, self._adc_conversion[idx][Y_MIN], self._adc_conversion[idx][Y_MAX], self._adc_conversion[idx][OFFSET], self._adc_conversion[idx][UNDER_X], self._adc_conversion[idx][OVER_X])

    def read_420(self, ch, raw=False, electric_value=False):
        '''

.. method:: read_420(ch, raw=False, electric_value=False)

        Read value from the 4-20mA ADC. It is possible to get the raw
        data from the ADC, or the electric value of the read signal, or by
        default it is converted with the rules defined using
        `set_conversion_010_420` method.

        :param ch: Select the ADC channel to read from. Can be one of 1, 2, 3, 4.
        :param raw: If set, the raw data of the ADC is returned. (D=False)
        :param electric_value: If set, the electric value is returned. (D=False)
        
        '''
        if raw and electric_value:
            raise UnsupportedError
        if ch not in (1,2,3,4):
            raise ValueError
        idx = ch - 1
        channel = ch + 3
        self.lockI2C.acquire()
        self.adc.set(os=0, ch=channel, pga=self._adc_config[idx][PGA], sps=self._adc_config[idx][SPS])
        data = self.adc.get_raw_data()
        self.lockI2C.release()
        if raw:
            return data
        edata = self._convert_420(data, self._adc_config[idx][VREF])
        if electric_value:
            return edata
        return self._scale_010_420(edata, 4, 20, self._adc_conversion[idx][Y_MIN], self._adc_conversion[idx][Y_MAX], self._adc_conversion[idx][OFFSET], self._adc_conversion[idx][UNDER_X], self._adc_conversion[idx][OVER_X])
#-endif


#-if ADC_RESISTIVE
    def _init_adc_resistive(self):
        # resistive adc
        self.lockI2C.acquire()
        self.adc_resistive = ads1015.ADS1015(I2C0, addr=72, clk=self.i2c_clk)
        self._adc_resistive_config = [
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]},
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]},
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]},
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]}
        ]
        self._adc_resistive_conversion = [
            {V_MIN: 0, REF_TABLE: [], DELTA: 0, OFFSET: 0, OUT_OF_RANGE: None},
            {V_MIN: 0, REF_TABLE: [], DELTA: 0, OFFSET: 0, OUT_OF_RANGE: None},
            {V_MIN: 0, REF_TABLE: [], DELTA: 0, OFFSET: 0, OUT_OF_RANGE: None},
            {V_MIN: 0, REF_TABLE: [], DELTA: 0, OFFSET: 0, OUT_OF_RANGE: None}
        ]
        self.lockI2C.release()


    def config_adc_resistive(self, ch, pga, sps):
        '''

.. method:: config_adc_resistive(ch, pga, sps)

        This method configure one of the four resistive ADC channels
        (chosen using the `ch` parameter).

        :param ch: Chose the channel to be configured (can be 1, 2, 3, 4).
        :param pga: Set the PGA Gain.
        :param sps: Set the samples-per-second data rate.
        
        '''
        # config adc resistive channels
        if ch not in (1,2,3,4):
            raise ValueError
        self._check_adc_config(pga, sps)
        idx = ch - 1
        self._adc_resistive_config[idx][PGA]  = pga
        self._adc_resistive_config[idx][SPS]  = sps
        self._adc_resistive_config[idx][VREF] = VREF_ADS1015[pga]


    def set_conversion_resistive(self, ch, v_min, ref_table, delta, offset=0, out_of_range=None):
        '''

.. method:: set_conversion_resistive(ch, v_min, ref_table, delta, offset=0, out_of_range=None)

        Set the conversion method to be applied for the data read from the
        resistive ADC. The conversion table can be configured using the
        `v_min`, `ref_table` and `delta` parameters.
        `out_of_range` is the value returned when the conversion exceed the bounds of
        the lookup table, defaults to None.

        :param ch: Chose the channel to be configured (can be 1, 2, 3, 4).
        :param v_min: Set the minimum value of the table.
        :param ref_table: List of numbers representing the lookup table values to be used for scale.
        :param delta: Step between two adjacent element of the table.
        :param offset: Optionally set an offset to be applied. (D=0)
        :param out_of_range: Value returned if a value can't be found on the table. (D=None)
        
        '''
        # if out_of_range is None: out_of_range = v_min
        if ch not in (1,2,3,4):
            raise ValueError
        idx = ch - 1
        self._adc_resistive_conversion[idx][V_MIN]        = v_min
        self._adc_resistive_conversion[idx][REF_TABLE]    = ref_table
        self._adc_resistive_conversion[idx][DELTA]        = delta
        self._adc_resistive_conversion[idx][OFFSET]       = offset
        self._adc_resistive_conversion[idx][OUT_OF_RANGE] = out_of_range


    def read_resistive(self, ch, raw=False, electric_value=False):
        '''

.. method:: read_resistive(ch, raw=False, electric_value=False)

        Read value from the resistive ADC. It is possible to get the raw
        data from the ADC, or the electric value of the read signal, or by
        default it is converted with the rules defined using
        `set_conversion_resistive` method.

        :param ch: Select the ADC channel to read from. Can be one of 1, 2, 3, 4.
        :param raw: If set, the raw data of the ADC is returned. (D=False)
        :param electric_value: If set, the electric value is returned. (D=False)
        
        '''
        if raw and electric_value:
            raise UnsupportedError
        if ch not in (1,2,3,4):
            raise ValueError
        idx = ch - 1
        channel = ch + 3
        self.lockI2C.acquire()
        self.adc_resistive.set(os=0, ch=channel, pga=self._adc_resistive_config[idx][PGA], sps=self._adc_resistive_config[idx][SPS])
        data = self.adc_resistive.get_raw_data()
        self.lockI2C.release()
        if raw:
            return data
        edata = self._convert_resistive(data, self._adc_resistive_config[idx][VREF])
        if electric_value:
            return edata
        return self._scale_resistive(edata, self._adc_resistive_conversion[idx][V_MIN], self._adc_resistive_conversion[idx][REF_TABLE], self._adc_resistive_conversion[idx][DELTA], self._adc_resistive_conversion[idx][OFFSET], self._adc_resistive_conversion[idx][OUT_OF_RANGE])

#-endif


#-if ADC_CURRENT
    def _init_adc_current(self):
        # electric current adc
        self.lockI2C.acquire()
        self.adc_current = ads1015.ADS1015(I2C0, addr=75, clk=self.i2c_clk)
        self._adc_current_config = [
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]},
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]},
            {PGA: 2, SPS: 7, VREF: VREF_ADS1015[2]}
        ]
        self._adc_current_conversion = [
            {N_SAMPLES: 400, NCOIL: 1, RATIO: 2000, VOLTAGE: 220, OFFSET: 0},
            {N_SAMPLES: 400, NCOIL: 1, RATIO: 2000, VOLTAGE: 220, OFFSET: 0},
            {N_SAMPLES: 400, NCOIL: 1, RATIO: 2000, VOLTAGE: 220, OFFSET: 0}
        ]
        self.lockI2C.release()


    def config_adc_current(self, ch, pga, sps):
        '''

.. method:: config_adc_current(ch, pga, sps)

        This method configure one of the three current ADC channels.

        :param ch: Chose the channel to be configured (can be 1, 2, 3).
        :param pga: Set the PGA Gain.
        :param sps: Set the samples-per-second data rate.
        
        '''
        # config adc current channels
        if ch not in (1,2,3):
            raise ValueError
        self._check_adc_config(pga, sps)
        idx = ch - 1
        self._adc_current_config[idx][PGA]  = pga
        self._adc_current_config[idx][SPS]  = sps
        self._adc_current_config[idx][VREF] = VREF_ADS1015[pga]


    def set_conversion_current(self, ch, n_samples=400, ncoil=1, ratio=2000, voltage=220, offset=0):
        '''
        
.. method:: set_conversion_current(ch, n_samples=400, ncoil=1, ratio=2000, voltage=220, offset=0)

        This method set the conversion parameters of a channel of the
        current ADC. It is possible to configure the numbr of samples that
        must be acquired, the number of coils done around the sensor, the
        ratio of the external sensor, and an offset.

        :param ch: Chose the channel to be configured (can be 1, 2, 3).
        :param n_samples: Set the number of samples to be read before conversion. (D=400)
        :param ncoil: Set the number of coils around the sensor. (D=1)
        :param ratio: Set the ratio current acquired by the sensor. (D=2000)
        :param voltage: Set the voltage of the current. (D=220)
        :param offset: Set an offset for the read data. (D=0)
        
        '''
        # if out_of_range is None: out_of_range = v_min
        if ch not in (1,2,3):
            raise ValueError
        idx = ch - 1
        self._adc_current_conversion[idx][N_SAMPLES] = n_samples
        self._adc_current_conversion[idx][NCOIL]    = ncoil
        self._adc_current_conversion[idx][RATIO]    = ratio
        self._adc_current_conversion[idx][VOLTAGE]  = voltage
        self._adc_current_conversion[idx][OFFSET]   = offset


    def read_power(self, ch, raw=False, electric_value=False):
        '''

.. method:: read_power(ch, raw=False, electric_value=False)

        Read value from the power ADC. It is possible to get the raw
        data from the ADC, or the electric value of the read signal, or by
        default it is converted with the rules defined using
        `set_conversion_current` method.

        :param: ch: Select the ADC channel to read from. Can be one of 1, 2, 3.
        :param: raw: If set, the raw data of the ADC is returned. (D=False)
        :param: electric_value: If set, the electric value is returned. (D=False)
        
        '''
        if raw and electric_value:
            raise UnsupportedError
        if ch not in (1,2,3):
            raise ValueError
        idx = ch - 1
        channel = ch + 3
        M = 0
        m = 5000
        self.lockI2C.acquire()
        self.adc_current.set(os=0, ch=channel, pga=self._adc_current_config[idx][PGA], sps=self._adc_current_config[idx][SPS])
        for i in range(self._adc_current_conversion[idx][N_SAMPLES]):
            rd = self.adc_current.get_raw_data()
            if rd > M:
                M = rd
            if rd < m:
                m = rd
        self.lockI2C.release()
        if raw:
            return M-m
        edata = self._convert_current(M-m, self._adc_current_config[idx][VREF], self._adc_current_conversion[idx][NCOIL], self._adc_current_conversion[idx][RATIO])
        if electric_value:
            return edata
        return self._convert_power(edata, self._adc_current_conversion[idx][VOLTAGE], self._adc_current_conversion[idx][OFFSET])
#-endif


#-if CAN_ENABLED
    def _init_can(self):
        # can init
        self.can = mcp2515.MCP2515(SPI0, D12, clk=self.spi_clk)
#-endif
#-if RS485_ENABLED
    def _init_rs485(self):
        # rs485 init
        self.tim = None
        self.rs485 = streams.serial(drvname=SERIAL1, baud=BAUD_485, set_default=False)
#-endif
#-if RS232_ENABLED
    def _init_rs232(self):
        # rs232 init
        self.rs232 = streams.serial(drvname=SERIAL0, baud=BAUD_232, set_default=False)
#-endif

#-if DEBUG_FZB
    # define locked print
    def FZprint(self, *args):
        print(*args)
#-endif

    def set_led(self, color):
        '''

.. method:: set_led(color)

        Set the LED status to a custom color.

        :param color: Character representing a color, see the table below.

        ==== =======
        Char Color
        ==== =======
        R    Red
        G    Green
        B    Blue
        M    Magenta
        Y    Yellow
        C    Cyan
        W    White
        ==== =======

        '''
        if color not in LEDS:
            raise ValueError
        self.lockI2C.acquire()
        for p in LEDS['W']:
            gpio.high(p)
        for p in LEDS[color]:
            gpio.low(p)
        self.lockI2C.release()

    def clear_led(self):
        '''

.. method:: clear_led()

        Clear LED status.

        '''
        self.lockI2C.acquire()
        for p in LEDS['W']:
            gpio.high(p)
        self.lockI2C.release()

    def pulse(self, color, duration):
        '''

.. method:: pulse(color, duration)

        Turn on the LED with the chosen color, then turn it off after the
        specified amount of time.

        :param color: Char representing a color. See `set_led` for a list.
        :param duration: Amount of time in milliseconds.

        '''
        if color not in LEDS:
            raise ValueError
        self.lockI2C.acquire()
        for p in LEDS[color]:
            gpio.low(p)
        self.lockI2C.release()
        sleep(duration)
        self.lockI2C.acquire()
        for p in LEDS['W']:
            gpio.high(p)
        self.lockI2C.release()

    def reverse_pulse(self, color, duration):
        '''

.. method:: reverse_pulse(color, duration)

        Wait the specified amount of time, the turn the LED on with the
        selected color.

        :param color: Char representing a color. See `set_led` for a list.
        :param duration: Amount of time in milliseconds.

        '''
        if color not in LEDS:
            raise ValueError
        self.lockI2C.acquire()
        for p in LEDS['W']:
            gpio.high(p)
        self.lockI2C.release()
        sleep(duration)
        self.lockI2C.acquire()
        for p in LEDS[color]:
            gpio.low(p)
        self.lockI2C.release()

    def error_sys(self):
        '''

.. method:: error_sys()

        Set the LED status to a system error (Red at low frequency).
        
        '''
        #-if DEBUG_FZB
        self.FZprint("led error_sys")
        #-endif
        self.lockI2C.acquire()
        try:
            for i in range(5):
                for p in LEDS['R']:
                    gpio.low(p)
                sleep(500)
                for p in LEDS['W']:
                    gpio.high(p)
                sleep(500)
        except Exception as e:
            #-if DEBUG_FZB
            print(e)
            #-endif
            #-if DEBUG_FZB
            self.FZprint(e)
            #-endif
            pass
        self.lockI2C.release()

    def error_connect(self):
        '''

.. method:: error_connect()

        Set the LED status to a net connection error (Red at high frequency).
        
        '''
        #-if DEBUG_FZB
        self.FZprint("led error_connect")
        #-endif
        self.lockI2C.acquire()
        try:
            for i in range(25):
                for p in LEDS['R']:
                    gpio.low(p)
                sleep(100)
                for p in LEDS['W']:
                    gpio.high(p)
                sleep(100)
        except Exception as e:
            #-if DEBUG_FZB
            print(e)
            #-endif
            #-if DEBUG_FZB
            self.FZprint(e)
            #-endif
            pass
        self.lockI2C.release()

    def error_custom(self, color, duration, n_pulses):
        '''

.. method:: error_custom(color, duration, n_pulses)

        Set the LED status to a custom error, specifying color, duration,
        and the numer of pulses.

        :param color: Char representing a color. See `set_led` for a list.
        :param duration: Amount of time in milliseconds for each pulse.
        :param n_pulses: Number of performed pulses.

        '''
        if color not in LEDS:
            raise ValueError
        #-if DEBUG_FZB
        self.FZprint("led error_custom")
        #-endif
        self.lockI2C.acquire()
        try:
            for i in range(n_pulses):
                for p in LEDS[color]:
                    gpio.low(p)
                sleep(duration)
                for p in LEDS['W']:
                    gpio.high(p)
                sleep(duration)
        except Exception as e:
            #-if DEBUG_FZB
            print(e)
            #-endif
            #-if DEBUG_FZB
            self.FZprint(e)
            #-endif
            pass
        self.lockI2C.release()

    def shut_down(self):
        '''

.. method:: shut_down()

        Turn off every external peripheral of the board. This includes the
        the three ADC, etc.
        
        '''
        sleep(100)
        gpio.low(D4)
        sleep(2000)

    def power_on(self):
        '''
        
.. method:: power_on()

        Turn back on the external peripherals, such as ADCs.
        
        '''
        sleep(100)
        gpio.high(D4)
        sleep(2000)

    # turn on relay
    def relay_on(self, n_rel):
        """

.. method:: relay_on(n_rel)

        Switch the selected relay ON, COM contact is closed on NO contact.

        :param n_rel: Relay to be turned on, possible values are 1 or 2.

        """
        if n_rel == 1:
            rel = REL_1
        elif n_rel == 2:
            rel = REL_2
        else:
            raise ValueError
        self.lockI2C.acquire()
        gpio.high(rel)
        self.lockI2C.release()

    # turn off relay
    def relay_off(self, n_rel):
        """

.. method:: relay_off(n_rel)

        Switch the selected relay OFF, COM contact is closed on NC contact.

        :param n_rel: Relay to be turned off, possible values are 1 or 2.

        """
        if n_rel == 1:
            rel = REL_1
        elif n_rel == 2:
            rel = REL_2
        else:
            raise ValueError
        self.lockI2C.acquire()
        gpio.low(rel)
        self.lockI2C.release()

    # turn on sink
    def sink_on(self, n_snk):
        """

.. method:: sink_on(n_snk)

        Switch the selected sink ON, Sink channel is shorted to GND.

        :param n_snk: Sink to be turned on, possible values are 1 or 2.

        """
        if n_snk == 1:
            snk = SNK_1
        elif n_snk == 2:
            snk = SNK_2
        else:
            raise ValueError
        self.lockI2C.acquire()
        gpio.high(snk)
        self.lockI2C.release()

    # turn off sink
    def sink_off(self, n_snk):
        """

.. method:: sink_off(n_snk)

        Switch the selected sink OFF, Sink channel is shorted to GND.

        :param n_snk: Sink to be turned off, possible values are 1 or 2.
        
        """
        if n_snk == 1:
            snk = SNK_1
        elif n_snk == 2:
            snk = SNK_2
        else:
            raise ValueError
        self.lockI2C.acquire()
        gpio.low(snk)
        self.lockI2C.release()

    # get opto value
    def get_opto(self, n_opto):
        '''

.. method:: get_opto(n_opto)

        Get the value read from the selected opto.

        :param n_opto: Opto to read data from. Possible values are 1 or 2.

        '''
        # return 0 if signal on opto is low, otherwise return 1 if signal on opto is high
        if n_opto == 1:
            opto = ISO_1
        elif n_opto == 2:
            opto = ISO_2
        else:
            raise ValueError
        self.lockI2C.acquire()
        val = 1 - gpio.get(opto)
        self.lockI2C.release()
        return val

#-if FZB_V7
    # get battery status [charging, discharging or charged]
    def get_battery_status(self):
        self.lockI2C.acquire()
        bat = gpio.get(BATT)
        ext = gpio.get(EXTV)
        self.lockI2C.release()
        if ext and bat:
            return "charged"
        elif ext:
            return "charging"
        return "discharging"
#-endif

    # get power source [external or battery]
    def get_power_source(self):
        '''

.. method:: get_power_source(n_opto)

        Get a string representing the alimentation source for the box.
        The returned string can be one of "external" or "battery".
        
        '''
        self.lockI2C.acquire()
        ext = gpio.get(EXTV)
        self.lockI2C.release()
        if ext:
            return "external"
        return "battery"

    # scale adc_010_420 values and add offset
    def _scale_010_420(self, value, x_min, x_max, y_min, y_max, offset, under_x, over_x):
        if value < x_min:
            return under_x
        if value > x_max:
            return over_x
        return y_min + (value - x_min) * (y_max - y_min) / (x_max - x_min) + offset

    # convert raw 4-20 values
    def _convert_420(self, raw, vref):
        G = 5 # ad8277's inverse gain
        R = 124 # on board sense resistor
        V = (raw/2047) * vref * G
        c = V/R
        return c*1000

    # convert raw 0-10 values
    def _convert_010(self, raw, vref):
        G = 5 # ad8277's inverse gain
        V = (raw/2047) * vref * G
        return V

    # scale resistive values and add offset
    def _scale_resistive(self, value, v_min, ref_table, delta, offset, out_of_range):
        if ref_table[0] < ref_table[-1]:
            ref_table = ref_table[::-1]
        if value >= ref_table[0]:
            return out_of_range
        for i in range(len(ref_table)-1):
            if value < ref_table[i+1]: # iterate until the value is between two ref_table entries
                continue
            x1 = ref_table[i]
            y1 = v_min + (delta*i)
            x2 = ref_table[i+1]
            y2 = v_min + (delta*(i+1))
            m = (y2-y1)/(x2-x1)
            q = (x2*y1 - x1*y2)/(x2-x1)
            value = m*value + q
            break
        else:
            return out_of_range
        return value

    # convert raw resistive values
    def _convert_resistive(self, raw, vref):
        Rpu = 43 # pullup on board
        Vdd = 3.3 # vdd
        V = (raw/2047)*vref
        value = V * Rpu / (Vdd-V)
        return value

    #Effective value
    def _convert_power(self, value, voltage=220, offset=0):
        power = (value*voltage*0.7071)+offset
        return power

    # convert raw current values
    def _convert_current(self, raw, vref, ncoil=1, ratio=2000):
        i_secondary = (raw/2)*vref/2048/20  #(Vmax - Vmin)/rangeadc*Vref/rsense
        i_primary = ratio*i_secondary/ncoil
        return i_primary

    def net_init(self, static_ip=None, static_mask=None, static_gateway=None, static_dns=None):
        '''

.. method:: net_init(static_ip=None, static_mask=None, static_gateway=None, static_dns=None)

        Initialize network driver. If static ip and static mask are set, DHCP
        is disabled and static ip, mask, gateway, and dns are used instead (ignored for gsm connectivity).

        :param static_ip: Static IP address of the device. (D=None)
        :param static_mask: Static subnet mask. (D=None)
        :param static_gateway: Static gateway address. (D="0.0.0.0")
        :param static_dns: Static DNS server address. (D="8.8.8.8")

        '''
        if not self.net_driver:
            raise InvalidHardwareStatusError

        if self.net_initialized:
            #-if DEBUG_FZB
            self.FZprint("network driver already initialized")
            #-endif
            raise NetworkInitializedException
        try:
            self.net_driver = self.net_driver.init()
            if 'gsm' not in self.check_net:
                if static_ip and static_mask:
                    if not static_gateway:
                        static_gateway = '0.0.0.0'
                    if not static_dns:
                        static_dns = '8.8.8.8'
                    self.net.set_link_info(static_ip, static_mask, static_gateway, static_dns)
            self.net_initialized = True
        except Exception as e:
            #-if DEBUG_FZB
            self.FZprint("Error on network init")
            ##-if NETWORK_GSM
            self.FZprint("check SW1 position 1-3-5-11 on for slo1; 2-4-6-12 on for slot2 (preferred)")
            ##-endif
            #-endif
            self.error_sys()
            raise NetworkInitException

    def _net_info(self):
        info = self.net.link_info()
        if len(info) > 2:
            self.net_info = {
                "ip":info[0],
                "msk":info[1],
                "gtw":info[2],
                "dns":info[3]
            }
        else:
            self.net_info = {
                "ip":info[0],
                "dns":info[1],
            }
        #-if DEBUG_FZB
        for k, v in self.net_info.items():
            self.FZprint(" - ",k,":",v)
        #-endif

#-if NETWORK_ETH
    def net_connect(self):
        '''

.. method:: net_connect() - Ethernet

        Connect to the network using the Ethernet interface. This method is
        enabled at compilation time using the `NETWORK_ETH` flag.

        .. warning:: use only one of `NETWORK_ETH`, `NETWORK_WIFI`, `NETWORK_GSM`.

        
        '''
        ##-if DEBUG_FZB
        self.FZprint("eth driver")
        ##-endif
        for _ in range(5):
            try:
                self.net.link()
                self._eth_en = True
                self._net_info()
                break
            except Exception as e:
                ##-if DEBUG_FZB
                print(e)
                ##-endif
                ##-if DEBUG_FZB
                self.FZprint("... failed")
                self.FZprint("eth connect", e)
                ##-endif
                pass
        else:
            ##-if DEBUG_FZB
            self.FZprint("Error in eth connect")
            ##-endif
            self.error_connect()
            raise NetworkConnectionException
#-endif
#-if NETWORK_WIFI
    def net_connect(self, ssid, password, keying=None):
        '''

.. method:: net_connect(ssid, password, keying=None) - Wi-Fi

        Connect to the network using the Wi-Fi interface. This method is
        enabled at compilation time using the `NETWORK_WIFI` flag.

        .. warning:: use only one of `NETWORK_ETH`, `NETWORK_WIFI`, `NETWORK_GSM`.

        :param ssid: SSID of the Wi-fi network to connect to.
        :param password: Password of the Wi-Fi network.
        :param keying: Encryption type. (D=None)

        '''
        ##-if DEBUG_FZB
        self.FZprint("wifi driver")
        ##-endif
        for _ in range(5):
            try:
                if keying == None:
                    keying = self.net.WIFI_WPA2
                self.net.link(ssid, keying, password)
                self._wifi_en = True
                self._net_info()
                break
            except Exception as e:
                ##-if DEBUG_FZB
                print(e)
                ##-endif
                ##-if DEBUG_FZB
                self.FZprint("... failed")
                self.FZprint("wifi connect", e)
                ##-endif
                pass
        else:
            ##-if DEBUG_FZB
            self.FZprint("Error in wifi connect")
            ##-endif
            self.error_connect()
            raise NetworkConnectionException
#-endif
#-if NETWORK_GSM
    def net_connect(self, apn):
        '''

.. method:: net_connect(apn) - GSM

        Connect to the network using the GSM/GPRS interface. This method is
        enabled at compilation time using the `NETWORK_GSM` flag.

        .. warning:: use only one of `NETWORK_ETH`, `NETWORK_WIFI`, `NETWORK_GSM`.

        :param apn: APN name of the SIM operator to connect to.
        
        '''
        ##-if DEBUG_FZB
        self.FZprint("gsm driver")
        ##-endif
        for _ in range(5):
            try:
                self.net.attach(apn)
                self._gsm_en = True
                self._net_info()
                break
            except Exception as e:
                ##-if DEBUG_FZB
                print(e)
                ##-endif
                ##-if DEBUG_FZB
                self.FZprint("... failed")
                self.FZprint("gsm connect", e)
                ##-endif
                pass
        else:
            ##-if DEBUG_FZB
            self.FZprint("Error in gsm connect")
            ##-endif
            self.error_connect()
            raise NetworkConnectionException
#-endif

    def get_rssi(self):
        '''

.. method:: get_rssi()

        Get RSSI of the current wireless connection (works only for Wi-Fi and GSM).
        
        '''
        if self._eth_en:
            raise UnsupportedError
        return self.net.rssi()

    def is_linked(self):
        if self._wifi_en or self._eth_en:
            return self.net.is_linked()

#-if RS485_ENABLED
    def _rs485_transmit(self):
        self.lockI2C.acquire()
        gpio.high(RS485EN)
        self.lockI2C.release()

    def _rs485_receive(self):
        self.lockI2C.acquire()
        gpio.low(RS485EN)
        self.lockI2C.release()

    def read_rs485(self, timeout=3000):
        '''

.. method:: read_rs485(timeout=3000)

        Read from the RS485 peripheral (it must be enabled in the class
        constructor).
        After the timeout expires, None is returned.

        :param timeout: Maximum timeout to wait for a message in milliseconds. (D=3000)

        '''
        self._rs485_receive()
        if self.tim == None:
            self.tim = timers.timer()
            self.tim.start()
        else:
            self.tim.reset()
        # self.lockSERIAL1.acquire()
        while not self.rs485.available():
            if self.tim.get() > timeout:
                # self.lockSERIAL1.release()
                break
            sleep(1)
        sleep(10)
        msg = []
        n = self.rs485.available()
        if n>0:
            msg = self.rs485.read(n)
        # self.lockSERIAL1.release()
        return msg
    
    def write_rs485(self, msg):
        '''
        
.. method:: write_rs485(msg)

        Write to the RS485 peripheral (it must be enabled in the class
        constructor).

        :param msg: Message to be sent.
        
        '''
        self._rs485_transmit()
        # self.lockSERIAL1.acquire()
        self.rs485.write(msg)
        # self.lockSERIAL1.release()
#-endif

#-if RS232_ENABLED
    def read_rs232(self, timeout=3000):
        '''

.. method:: read_rs232(timeout=3000)

        Read from the RS232 peripheral (it must be enabled in the class
        constructor).
        After the timeout expires, None is returned.

        :param timeout: Maximum timeout to wait for a message in milliseconds. (D=3000)

        '''
        t = timers.timer()
        # self.lockSERIAL0.acquire()
        t.start()
        while not self.rs232.available():
            if t.get() > timeout:
                # self.lockSERIAL0.release()
                return None
            sleep(1)
        n = self.rs232.available()
        msg = self.rs232.read(n)
        # self.lockSERIAL0.release()
        return msg

    def write_rs232(self, msg):
        '''

.. method:: write_rs232(msg)

        Write to the RS232 peripheral (it must be enabled in the class
        constructor).

        :param msg: Message to be sent.

        '''
        # self.lockSERIAL0.acquire()
        self.rs232.write(msg)
        # self.lockSERIAL0.release()
#-endif

    def can_init(self, idmode, speed, clock):
        '''

.. method:: can_init(idmode, speed, clock)

        Initialize the CAN interface.

        :param idmode: Set the RX buffer id mode (selectable from mcp2515.MCP_STDEXT, mcp2515.MCP_STD, mcp2515.MCP_EXT, or mcp2515.MCP_ANY).
        :param speed: Set the speed of the CAN communication.
        :param clock: Set the clock of the CAN Communication.
        
        '''
#-if CAN_ENABLED
        self.lockSPI.acquire()
        self.can.init(idmode, speed, clock)
        self.lockSPI.release()
#-else
        ##-if DEBUG_FZB
        print("CAN disabled! Enable it in project.yml")
        ##-endif
        raise UnsupportedError
#-endif
#-if CAN_ENABLED
    def set_can_mode(self, mode):
        '''

.. method:: set_can_mode(mode)

        Set CAN to the specified mode.

        :param mode: The mode to be set. One of ("NORMAL", "SLEEP", "LOOPBACK", "LISTENONLY", "CONFIG", "POWERUP", "ONE_SHOT").
        
        '''
        self.lockSPI.acquire()
        self.can.set_mode(mode)
        self.lockSPI.release()

    def can_init_mask(self, num, data, ext):
        '''

.. method:: can_init_mask(num, data, ext)

        Init CAN masks.

        :param num: 0 to set mask 0 on RX buffer, 1 to set mask 1 on RX buffer
        :param data: Data mask.
        :param ext: 0 for standard ID, 1 for Extended ID.
        
        '''
        self.lockSPI.acquire()
        self.can.init_mask(num, data, ext)
        self.lockSPI.release()

    def can_init_filter(self, num, data, ext):
        '''

.. method:: can_init_filter(num, data, ext)

        Init filters.

        Parameters:
        :param num: Number of filter to be set in RX buffer (from 0 to 5).
        :param data: Data filter.
        :param ext: 0 for standard ID, 1 for Extended ID.
        
        '''
        self.lockSPI.acquire()
        self.can.init_filter(num, data, ext)
        self.lockSPI.release()

    def can_send(self, canid, data, ext=None):
        '''

.. method:: can_send(canid, data, ext=None)

        Sends CAN messages.

        :param canid: ID of the CAN message (bytearray of 4 bytes).
        :param data: Data to be sent (list of 8 bytes).
        :param ext: 0 for standard ID, 1 for Extended ID (D=None, auto detected)

        '''
        self.lockSPI.acquire()
        self.can.send(canid, data, ext)
        self.lockSPI.release()

    def can_receive(self):
        '''

.. method:: can_receive()

        Receives CAN messages returning CAN id value and related data
        message.
        
        '''
        self.lockSPI.acquire()
        pkg = self.can.recv()
        self.lockSPI.release()
        return pkg
#-endif
    # SD
    def sd_init(self, mode):
        '''

.. method:: can_receive(mode)

        Initialize a SD card, using the SPI slot or the SD slot.

        :param mode: One of "SD", "SPI".

        '''
#-if SDCARD_ENABLED
        if self._spi_en and mode == "SD":
            raise UnsupportedError
        if mode == "SPI":
            self.lockSPI.acquire()
            fatfs.mount('0:', {"drv": SPI0, "cs": D32, "clock": self.spi_clk})
            self.lockSPI.release()
        elif mode == "SD":
            self._sd_en = True
            fatfs.mount('0:', {"drv": SD1, "freq_khz": SD_FREQ, "bits": 1})
        else:
            raise UnsupportedError
#-else
        ##-if DEBUG_FZB
        print("SD card support disabled! Enable it in project.yml")
        ##-endif
        raise UnsupportedError
#-endif

    # Internal Flash Management
    def flash_load(self, start_address, tot_size, rjson=False):
        '''

.. method:: flash_load(start_address, tot_size, rjson=False)

        Load a file from the internal flash memory.

        :param start_address: The address of the file beginning.
        :param tot_size: The size of the file to be read.
        :param r_json: If set, the file is parsed as a JSON and a dict return. (D=False)
        
        '''
        self.lockFLASH.acquire()
        info = None
        jinfo = None
        try:
            ff = flash.FlashFileStream(start_address, tot_size)
            info_len = ff.read_int()
            info = ff.read(info_len)
            if rjson:
                jinfo = json.loads(info)
        except Exception as e:
            #-if DEBUG_FZB
            print(e)
            #-endif
            #-if DEBUG_FZB
            self.FZprint("error in flash_load_json",e)
            #-endif
            pass
        self.lockFLASH.release()
        if rjson:
            return jinfo
        info.append(0x0)
        return info

    def flash_write_buff(self, start_address, tot_size, buff, seek=None):
        '''

.. method:: flash_write_buff(start_address, tot_size, buff, seek=None)

        Write a buffer in the internal flash memory, at the specified address.

        :param start_address: Initial address of the memory to be written.
        :param tot_size: Total size of the data to be written.
        :param buff: Buffer of data to be written.
        :param seek: Optional seek destionation to be done before writing. (D=None)
        
        '''
        self.lockFLASH.acquire()
        try:
            fs = flash.FlashFileStream(start_address, tot_size)
            if seek:
                fs.seek(seek)
            fs.write(buff)
            fs.flush()
        except Exception as e:
            #-if DEBUG_FZB
            print(e)
            #-endif
            #-if DEBUG_FZB
            self.FZprint("error in flash_write_buff")
            #-endif
            pass
        self.lockFLASH.release()

    def flash_read_buff(self, start_address, tot_size):
        '''

.. method:: flash_read_buff(start_address, tot_size)

        Return a buffer of data read at the specified start address.

        :param start_address: Initial address to read from.
        :param tot_size: Total size of data to be read.

        '''
        self.lockFLASH.acquire()
        try:
            fs = flash.FlashFileStream(start_address, tot_size)
        except Exception as e:
            #-if DEBUG_FZB
            print(e)
            #-endif
            #-if DEBUG_FZB
            self.FZprint("error in flash_read_buff")
            #-endif
            pass
        self.lockFLASH.release()
        return fs.bb

