from modbus import ModbusBridge
from sx1262 import SX1262, LoraConfig
import logging


if __name__ == "__main__":
    logging.basicConfig(
        format='%(asctime)s :: %(levelname)s :: %(message)s',
        datefmt='%Y.%m.%d %H:%M:%S',
        level=logging.INFO)

    lora = SX1262(resetPin=25)
    lora.tcxo = False
    lora.config.frequency = 868000000
    lora.config.spreadingFactor = 8
    lora.config.bandwidth = LoraConfig.LoraBw.bw125
    lora.config.codingRate = LoraConfig.LoraCr.cr4_5
    lora.config.preambleLength = 8
    lora.config.fixedPayloadLength = False
    lora.config.crc = True
    lora.simple_init()

    bridge = ModbusBridge(transport=lora)
    bridge.run()
