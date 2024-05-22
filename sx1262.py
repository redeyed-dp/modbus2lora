import logging
import spidev
from enum import IntEnum
from time import sleep

try:
    import RPi.GPIO as GPIO
    gpio_enabled = True
except:
    gpio_enabled = False
    logging.warning("RPi.GPIO module not available.")


class LoraCmd(IntEnum):
    setSleep = 0x84
    setStandby = 0x80
    setFs = 0xC1
    setTx = 0x83
    setRx = 0x82
    stopTimerOnPreamble = 0x9F
    setRxDutyCycle = 0x94
    setCad = 0xC5
    setTxContinuousWave = 0xD1
    setTxInfinitePreamble = 0xD2
    setRegulatorMode = 0x96
    calibrate = 0x89
    calibrateImage = 0x98
    setPaConfig = 0x95
    setRxTxFallbackMode = 0x93
    writeRegister = 0x0D
    readRegister = 0x1D
    writeBuffer = 0x0E
    readBuffer = 0x1E
    setDioIrqParams = 0x08
    getIrqStatus = 0x12
    clearIrqStatus = 0x02
    setDIO2AsRfSwitchCtrl = 0x9D
    setDIO3AsTcxoCtrl = 0x97
    setRfFrequency = 0x86
    setPacketType = 0x8A
    getPacketType = 0x11
    setTxParams = 0x8E
    setModulationParams = 0x8B
    setPacketParams = 0x8C
    setCadParams = 0x88
    setBufferBaseAddress = 0x8F
    setLoRaSymbNumTimeout = 0xA0
    getStatus = 0xC0
    getRssiInst = 0x15
    getRxBufferStatus = 0x13
    getPacketStatus = 0x14
    getDeviceErrors = 0x17
    clearDeviceErrors = 0x07
    getStats = 0x10
    resetStats = 0x00

class LoraReg(IntEnum):
    HoppingEnable = 0x0385
    PacketLength = 0x0386
    NbHoppingBlocks = 0x0387

    # Non-standard DIOx control
    DIOxOutputEnable = 0x0580
    DIOxInputEnable = 0x0583
    DIOxPullUpControl = 0x0584
    DIOxPullDownControl = 0x0585
    WhiteningInitialValueMSB = 0x06B8       # Initial value used for the whitening LFSR in FSK mode.
    WhiteningInitialValueLSB = 0x06B9
    CrcInitialValieMSB = 0x06BC             # Initial value used for the polynomial used to compute the CRC in FSK mode
    CrcInitialValieLSB = 0x06BD
    CrcPolynomialValueMSB = 0x06BE          # Polynomial used to compute the CRC in FSK mode
    CrcPolynomialValueLSB = 0x06BF
    SyncWord0 = 0x06C0                      # 1st byte of the Sync Word in FSK mode
    SyncWord1 = 0x06C1
    SyncWord2 = 0x06C2
    SyncWord3 = 0x06C3
    SyncWord4 = 0x06C4
    SyncWord5 = 0x06C5
    SyncWord6 = 0x06C6
    SyncWord7 = 0x06C7
    NodeAddress = 0x06CD                    # Node Address used in FSK mode
    BroadcastAddress = 0x06CE               # Broadcast Address used in FSK mode
    IQPolaritySetup = 0x0736                # Optimize the inverted IQ operation
    loraSyncWordMSB = 0x0740                # Differentiate the LoRa signal for Public or Private Network
    loraSyncWordLSB = 0x0741
    RandomNumberGen0 = 0x0819               # Can be used to get a 32-bit random number
    txModulation = 0x0889
    rxGain = 0x08AC
    TxClampConfig = 0x08D8
    OCPConfig = 0x08E7
    RTCControl = 0x0902
    XTATrim = 0x0911
    XTBTrim = 0x0912
    DIO3OutputVoltageControl = 0x920
    EventMask = 0x944

class LoraFallbackMode(IntEnum):
    fs = 0x40
    stdby_xosc = 0x30
    stdby_rc = 0x20

class LoraTcxoControl(IntEnum):
    v1_6 = 0x00
    v1_7 = 0x01
    v1_8 = 0x02
    v2_2 = 0x03
    v2_4 = 0x04
    v2_7 = 0x05
    v3_0 = 0x06
    v3_3 = 0x07

class LoraRamp(IntEnum):
    ramp10 = 0x00
    ramp20 = 0x01
    ramp40 = 0x02
    ramp80 = 0x03
    ramp200 = 0x04
    ramp800 = 0x05
    ramp1700 = 0x06
    ramp3400 = 0x07

class LoraConfig:
    class LoraBw(IntEnum):
        bw500 = 0x06
        bw250 = 0x05
        bw125 = 0x04
        bw062 = 0x03
        bw041 = 0x0A
        bw031 = 0x02
        bw020 = 0x09
        bw015 = 0x01
        bw010 = 0x08
        bw007 = 0x00

    class LoraCr(IntEnum):
        cr4_5 = 0x01
        cr4_6 = 0x02
        cr4_7 = 0x03
        cr4_8 = 0x04

    _frequency: int
    bandwidth: LoraBw
    codingRate: LoraCr
    _spreadingFactor: int
    preambleLength: int
    _payloadLength = 255
    fixedPayloadLength: bool
    crc: bool

    @property
    def frequency(self) -> int:
        return self._frequency

    @frequency.setter
    def frequency(self, freq: int):
        #assert 350000000 < freq < 550000000, "Unsupported frequency"
        self._frequency = freq

    @property
    def spreadingFactor(self) -> int:
        return self._spreadingFactor

    @spreadingFactor.setter
    def spreadingFactor(self, sf: int):
        assert 5 <= sf <= 12, "Unsupported spreading factor"
        self._spreadingFactor = sf

    @property
    def payloadLength(self) -> int:
        return self._payloadLength

    @payloadLength.setter
    def payloadLength(self, pl: int):
        assert 0 <= pl <= 255, "Invalid payload length"
        self._payloadLength = pl


class LoraIrq(IntEnum):
    radioNone = 0x0000
    txDone = 0x0001
    rxDone = 0x0002
    preambleDetected = 0x0004
    syncwordValid = 0x0008
    headerValid = 0x0010
    headerError = 0x0020
    crcError = 0x0040
    cadDone = 0x0080
    cadActivityDetected = 0x0100
    rxTxTimeout = 0x0200
    radioAll = 0xFFFF


class SX1262:
    def __init__(self, bus=0, device=0, resetPin=0):
        self.tcxo = False
        self.spi = spidev.SpiDev()
        self.spi.open(bus=bus, device=device)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0b00  # CPOL = 0, CPHA = 0
        self.spi.lsbfirst = False
        self.config = LoraConfig()
        if resetPin > 0:
            self.reset_pin = resetPin
        if gpio_enabled:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.reset_pin, GPIO.OUT)

    def reset(self):
        if gpio_enabled:
            GPIO.output(self.reset_pin, False)
            sleep(0.01)
            GPIO.output(self.reset_pin, True)
            sleep(0.01)
            logging.debug("Chip was rebooted")

    def writeCommand(self, command: LoraCmd, value):
        buf = [command]
        if type(value) is int:
            buf.append(value)
        else:
            buf.extend(value)
        self.spi.xfer2(buf)

    def readCommand(self, command: LoraCmd, size: int):
        buf = [command]
        buf.extend([0x00] * size)
        resp = self.spi.xfer2(buf)
        return resp[1:]

    def writeRegister(self, address: LoraReg, value):
        buf = [LoraCmd.writeRegister, address >> 8, address & 0xFF]
        if type(value) is int:
            buf.append(value)
        else:
            buf.extend(value)
        self.spi.xfer2(buf)

    def readRegister(self, address: LoraReg, size: int) -> int:
        buf = [LoraCmd.readRegister, address >> 8, address & 0xFF, 0x00]
        buf.extend([0x00] * size)
        resp = self.spi.xfer2(buf)
        return resp[4:]

    def writeBuffer(self, offset: int, data: bytes):
        buf = [offset]
        buf.extend(data)
        self.writeCommand(LoraCmd.writeBuffer, buf)  # WriteBuffer opcode
        sleep(0.001)

    def readBuffer(self, offset: int, size: int) -> bytes:
        buf = [LoraCmd.readBuffer, offset, 0x00]
        buf.extend([0x00] * size)
        resp = self.spi.xfer2(buf)
        return resp[3:]

    def setRegulatorMode(self, ldo=True):
        """
        Declares the power regulation used to power the device
        This command allows the user to specify if DC-DC or LDO is used for power regulation.
        Using only LDO implies that the Rx or Tx current is doubled
        """
        logging.debug(f"LoRa: Set regulator mode {'LDO' if ldo else 'DCDC'}")
        self.writeCommand(LoraCmd.setRegulatorMode, 0x00 if ldo else 0x01)
        sleep(0.01)

    def setRfSwitchMode(self, on=True):
        logging.debug("LoRa: Set DIO2 as RF switch")
        self.writeCommand(LoraCmd.setDIO2AsRfSwitchCtrl, 0x01 if on else 0x00)
        sleep(0.01)

    def calibrate(self, calibParam=0x7F):
        """ Calibrate all blocks ~ 3.5ms """
        logging.debug("LoRa: Calibrating all units")
        self.writeCommand(LoraCmd.calibrate, 0x7F)
        sleep(0.07)

    def setIrqParams(self, irqmask: LoraIrq):
        self.writeCommand(LoraCmd.setDioIrqParams, (irqmask >> 8, irqmask & 0xFF))
        sleep(0.001)

    def clearIrqStatus(self, irq: LoraIrq):
        self.writeCommand(LoraCmd.clearIrqStatus, (irq >> 8, irq & 0xFF))
        sleep(0.001)

    def getIrqStatus(self) -> int:
        resp = self.readCommand(LoraCmd.getIrqStatus, 3)
        return (resp[1] << 8) | resp[2]

    def setStandby(self, xosc=False):
        """
        Declares the oscillator in use while in standby mode
        Using the RC standby mode allow to reduce the energy consumption
        XOSC should be used for time critical applications
        """
        self.writeCommand(LoraCmd.setStandby, 0x01 if xosc else 0x00)
        sleep(0.01)

    def setSleep(self, sleepConfig: int):
        self.writeCommand(LoraCmd.setSleep, sleepConfig)
        sleep(0.001)

    def setRx(self, timeout: int, boosted=True):
        self.writeRegister(LoraReg.rxGain, 0x96 if boosted else 0x94)  # RX gain
        self.writeCommand(LoraCmd.setRx, ((timeout >> 16) & 0xFF, (timeout >> 8) & 0xFF, timeout & 0xFF))

    def setTx(self, timeout: int):
        self.writeCommand(LoraCmd.setTx, ((timeout >> 16) & 0xFF, (timeout >> 8) & 0xFF, timeout & 0xFF))

    def setDio3AsTcxoCtrl(self, voltage: LoraTcxoControl, timeout: int):
        logging.debug(f"LoRa: Set DIO3 as TCXO control")
        buf = (
            voltage & 0x07,
            (timeout >> 16) & 0xFF,
            (timeout >> 8) & 0xFF,
            timeout & 0xFF
        )
        self.writeCommand(LoraCmd.setDIO3AsTcxoCtrl, buf)
        sleep(0.01)

    def setPacketType(self, lora=True):
        logging.debug("LoRa: Set packet type LoRa")
        self.writeCommand(LoraCmd.setPacketType, 0x01 if lora else 0x00)
        sleep(0.005)

    def getPacketType(self) -> bool:
        resp = self.readCommand(LoraCmd.getPacketType, 2)
        return resp[-1] != 0

    def calibrateImage(self, freqMin: int, freqMax: int):
        logging.debug(f"LoRa: Calibrating image for band {freqMin} - {freqMax} MHz")
        self.writeCommand(LoraCmd.calibrateImage, (int(freqMin / 4), int(freqMax / 4)))
        sleep(0.07)

    def setPaConfig(self):
        self.writeCommand(LoraCmd.setPaConfig, (0x04, 0x07, 0x00, 0x01))

    def setRxTxFallbackMode(self, fallbackMode: LoraFallbackMode):
        self.writeCommand(LoraCmd.setRxTxFallbackMode, fallbackMode)

    def setOcp(self, current=140):
        assert current <= 140, "Maximum current for sx1262 is 140 mA"
        logging.debug(f"LoRa: Set overcurrent protection {current} mA")
        self.writeRegister(LoraReg.OCPConfig, int(current / 2.5))

    def setTxParams(self, power: int, rampTime: LoraRamp):
        assert -3 <= power <= 22, "Invalid power"
        logging.debug(f"LoRa: Set power {power} dBm")
        self.writeCommand(LoraCmd.setTxParams, (power, rampTime))
        sleep(0.02)

    def setModulationParamsLora(self):
        if self.config.bandwidth == LoraConfig.LoraBw.bw500:
            lowDatarateOptimize = 0x00
        elif self.config.bandwidth == LoraConfig.LoraBw.bw250:
            lowDatarateOptimize = 0x01 if self.config.spreadingFactor == 12 else 0x00
        elif self.config.bandwidth == LoraConfig.LoraBw.bw125:
            lowDatarateOptimize = 0x01 if self.config.spreadingFactor >= 11 else 0x00
        elif self.config.bandwidth == LoraConfig.LoraBw.bw062:
            lowDatarateOptimize = 0x01 if self.config.spreadingFactor >= 10 else 0x00
        elif self.config.bandwidth == LoraConfig.LoraBw.bw041:
            lowDatarateOptimize = 0x01 if self.config.spreadingFactor >= 9 else 0x00
        else:
            lowDatarateOptimize = 0x01
        logging.debug("LoRa: Set modulation parameters")
        self.writeCommand(LoraCmd.setModulationParams, (self.config.spreadingFactor, self.config.bandwidth, self.config.codingRate, lowDatarateOptimize))
        sleep(0.05)

    def setPacketParamsLora(self, invertIq=False):
        buf = (
            (self.config.preambleLength >> 8) & 0xFF,
            self.config.preambleLength & 0xFF,
            0x01 if self.config.fixedPayloadLength else 0x00,
            self.config.payloadLength,
            0x01 if self.config.crc else 0x00,
            0x01 if invertIq else 0x00
        )
        logging.debug("LoRa: Set packet parameters")
        self.writeCommand(LoraCmd.setPacketParams, buf)
        sleep(0.01)

    def setSyncWordLora(self, publicNetwork=False):
        syncword = 0x3444 if publicNetwork else 0x1424
        logging.debug(f"LoRa: Set sync word 0x{syncword:X}")
        self.writeRegister(LoraReg.loraSyncWordMSB, (syncword >> 8) & 0xFF)
        sleep(0.005)
        self.writeRegister(LoraReg.loraSyncWordLSB, syncword & 0xFF)
        sleep(0.005)

    def setBufferBaseAddresses(self, txBaseAddr: int, rxBaseAddr: int):
        assert 0 <= txBaseAddr <= 255, "Invalid TX base address"
        assert 0 <= rxBaseAddr <= 255, "Invalid RX base address"
        self.writeCommand(LoraCmd.setBufferBaseAddress, (txBaseAddr, rxBaseAddr))
        sleep(0.01)

    def getStatus(self) -> int:
        return self.readCommand(LoraCmd.getStatus, 1)

    def getRxBufferStatus(self) -> (int, int):
        """
        Returns the length of the last received packet (PayloadLengthRx)
        and the address of the first byte received (RxStartBufferPointer)
        """
        resp = self.readCommand(LoraCmd.getRxBufferStatus, 3)
        return resp[1:]

    def getPacketStatus(self) -> (int, int):
        resp = self.readCommand(LoraCmd.getPacketStatus, 3)
        rssi = -resp[1] // 2
        snr = resp[2] // 4
        return rssi, snr

    def getRssiInst(self) -> int:
        resp = self.readCommand(LoraCmd.getRssiInst, 2)
        return -resp[1] // 2

    def getDeviceErrors(self) -> int:
        resp = self.readCommand(LoraCmd.getDeviceErrors, 3)
        return (resp[1] << 8) | resp[2]

    def clearDeviceErrors(self):
        self.writeCommand(LoraCmd.clearDeviceErrors, 0x00)

    def setFrequency(self):
        freq = int(self.config.frequency * 33554432 / 32000000)
        buf = ((freq >> 24) & 0xFF, (freq >> 16) & 0xFF, (freq >> 8) & 0xFF, freq & 0xFF)
        logging.debug(f"LoRa: Set frequency {self.config.frequency:,} Hz")
        self.writeCommand(LoraCmd.setRfFrequency, buf)
        sleep(0.005)

    def simple_init(self):
        self.reset()
        self.setStandby()
        self.setRegulatorMode(ldo=True)
        self.setRfSwitchMode(on=True)
        if self.tcxo:
            self.setDio3AsTcxoCtrl(LoraTcxoControl.v3_3, 192)
        self.calibrate()
        if self.config.frequency < 500000000:
            self.calibrateImage(400, 480)
        else:
            self.calibrateImage(860, 960)
        self.setPacketType(lora=True)
        self.setPaConfig()
        self.setTxParams(power=22, rampTime=LoraRamp.ramp200)
        self.setOcp(current=140)
        self.setSyncWordLora(publicNetwork=False)
        self.setModulationParamsLora()
        self.setPacketParamsLora()
        self.setFrequency()
        self.setBufferBaseAddresses(0x00, 0x00)
        self.setStandby()

    def transmit(self, data: bytes):
        if self.config.fixedPayloadLength:
            assert len(data) == self.config.payloadLength, "Invalid payload length"
        elif self.config.payloadLength != len(data):
            self.config.payloadLength = len(data)
            self.setPacketParamsLora()
        self.setIrqParams(LoraIrq.txDone)
        self.writeBuffer(0x00, data)
        self.clearIrqStatus(LoraIrq.radioAll)
        self.setTx(0)
        for _ in range(3500):
            irq = self.getIrqStatus()
            if irq & LoraIrq.txDone:
                logging.debug(f"LoRa: {len(data)} bytes sent by {_}ms")
                sleep(0.001)
                return
            sleep(0.001)
        logging.debug(f"LoRa: Transmission does not complete")

    def receive(self) -> bytes:
        self.clearIrqStatus(LoraIrq.radioAll)
        self.setIrqParams(LoraIrq.rxDone | LoraIrq.crcError)
        self.setRx(0, True)
        for _ in range(3500):
            irq = self.getIrqStatus()
            if irq & LoraIrq.rxDone:
                if irq & LoraIrq.crcError:
                    return bytes()
                else:
                    size, offset = self.getRxBufferStatus()
                    return bytes(self.readBuffer(offset, size))
            sleep(0.001)
        return bytes()


