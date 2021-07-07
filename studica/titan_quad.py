from pyMobileRobotics.hal.can import CAN
from pyMobileRobotics.hal.can_receiver import CANReceiver
from enum import Enum
import struct


class TitanQuad():
    """Titan Quad Controller"""

    class __MessageType(Enum):
        SET_FREQ = 0x20C02A0
        DISABLE = 0x20C0020
        ENABLE = 0x20C0060
        PWM = 0x20C00A0
        ENC_M0 = 0x20C0960
        ENC_M1 = 0x20C09A0
        ENC_M2 = 0x20C09E0
        ENC_M3 = 0x20C0A20
        ENC_RST = 0x20C0360
        LSW = 0x20C0720

    __enabled = None
    __encCanRecv = [None, None, None, None]
    __encLastValue = [0, 0, 0, 0]

    def __init__(self, titanID=42, m0Frequency=15600, m1Frequency=15600, m2Frequency=15600, m3Frequency=15600):
        """
        Titan Quad

        Args:
            titanID (int, optional): Titan Quad ID. Defaults to 42.
            m0Frequency (int, optional): 馬達M0 PWM頻率. Defaults to 15600.
            m1Frequency (int, optional): 馬達M1 PWM頻率. Defaults to 15600.
            m2Frequency (int, optional): 馬達M2 PWM頻率. Defaults to 15600.
            m3Frequency (int, optional): 馬達M3 PWM頻率. Defaults to 15600.
        """
        self.__titanID = titanID
        self.__titanMsgID = TitanQuad.__getTitanIDToMsgID(self.__titanID)
        self.__encCanRecv[0] = CANReceiver(self.__getMsgID(TitanQuad.__MessageType.ENC_M0), 0xFFFFFFF, 2)
        self.__encCanRecv[1] = CANReceiver(self.__getMsgID(TitanQuad.__MessageType.ENC_M1), 0xFFFFFFF, 2)
        self.__encCanRecv[2] = CANReceiver(self.__getMsgID(TitanQuad.__MessageType.ENC_M2), 0xFFFFFFF, 2)
        self.__encCanRecv[3] = CANReceiver(self.__getMsgID(TitanQuad.__MessageType.ENC_M3), 0xFFFFFFF, 2)
        self.__lswCanRecv = CANReceiver(self.__getMsgID(TitanQuad.__MessageType.LSW), 0xFFFFFFF, 2)
        CAN.flushRxFIFO()
        CAN.flushTxFIFO()
        CAN.setMode(CAN.Mode.NORMAL)
        self.setFrequency(0, m0Frequency)
        self.setFrequency(1, m1Frequency)
        self.setFrequency(2, m2Frequency)
        self.setFrequency(3, m3Frequency)
        self.setDisabled()

    def setFrequency(self, motorID: int, frequency: int):
        if motorID not in range(0, 4):
            raise ValueError("Incorrect Motor ID.")
        freqPpack = struct.pack('<i', frequency)
        message = [
            motorID,
            freqPpack[0],
            freqPpack[1],
            freqPpack[2],
            freqPpack[3],
            0x00, 0x00, 0x00
        ]
        message = bytearray(message)
        CAN.sendMessage(self.__getMsgID(TitanQuad.__MessageType.SET_FREQ), message, periodMS=0)

    def setDisabled(self):
        """
        將Titan Quad設置為禁用狀態

        **禁用狀態下，將無法控制電機旋轉。**
        """
        if self.__enabled or self.__enabled == None:
            message = bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            CAN.sendMessage(self.__getMsgID(TitanQuad.__MessageType.DISABLE), message, periodMS=50)
            CAN.sendMessage(self.__getMsgID(TitanQuad.__MessageType.ENABLE), message, periodMS=0)
            self.__enabled = False

    def setEnabled(self):
        """
        將Titan Quad設置為啟用狀態"""
        if not(self.__enabled):
            message = bytearray([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            CAN.sendMessage(self.__getMsgID(TitanQuad.__MessageType.DISABLE), message, periodMS=0)
            CAN.sendMessage(self.__getMsgID(TitanQuad.__MessageType.ENABLE), message, periodMS=50)
            self.__enabled = True

    def setSpeed(self, motorID: int, speed: float):
        """
        設置電機速度

        Args:
            motorID (int): 電機ID
            speed (float): 電機速度(-1~1)
        """
        dutyCycle = 1 if speed > 1 else speed
        dutyCycle = -1 if speed < -1 else speed
        dutyCycle = abs(dutyCycle)
        inA = True if speed >= 0 else False
        inB = True if speed <= 0 else False
        self.setPWM(motorID, dutyCycle, inA, inB)

    def setPWM(self, motorID: int, dutyCycle: float, inA: bool, inB: bool):
        """
        設置電機PWM數值

        Args:
            motorID (int): 電機ID
            dutyCycle (int): 佔空比(0~1)
            inA (bool): PWM A控制
            inB (bool): PWM B控制

        Raises:
            ValueError: 電機ID錯誤
        """
        if motorID not in range(0, 4):
            raise ValueError("Incorrect Motor ID.")
        dutyCycle = 1 if dutyCycle > 1 else dutyCycle
        dutyCycle = 0 if dutyCycle < 0 else dutyCycle
        dutyCycle = int(dutyCycle * 255)
        dutyCyclePpack = struct.pack('<B', dutyCycle)
        message = [
            motorID,
            dutyCyclePpack[0],
            int(inA),
            int(inB),
            0x00, 0x00, 0x00, 0x00
        ]
        message = bytearray(message)
        CAN.sendMessage(self.__getMsgID(TitanQuad.__MessageType.PWM), message, periodMS=0)

    def getEncoderValue(self, motorID: int) -> int:
        """
        獲取編碼器數值

        Args:
            motorID (int): 電機ID

        Raises:
            ValueError: 電機ID錯誤

        Returns:
            int: 編碼器數值
        """
        if not(motorID in range(0, 4)):
            raise ValueError("Incorrect Motor ID.")
        canReceiver = self.__encCanRecv[motorID]
        msgNum, _, _, data, _ = canReceiver.readMessage()
        if msgNum > 0:
            encValuePack = [data[i] for i in range(4)]
            encValuePack = bytes(encValuePack)
            encValue = struct.unpack('<i', encValuePack)[0]
            self.__encLastValue[motorID] = encValue
            return encValue
        else:
            return self.__encLastValue[motorID]

    def resetEncoder(self, motorID: int):
        """
        重置編碼器

        Args:
            motorID (int): 電機ID

        Raises:
            ValueError: 電機ID錯誤
        """
        if motorID not in range(0, 4):
            raise ValueError("Incorrect Motor ID.")
        message = [
            motorID,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        ]
        message = bytearray(message)
        CAN.sendMessage(self.__getMsgID(TitanQuad.__MessageType.ENC_RST), message, periodMS=0)

    def getLimitSwitch(self, motorID: int):
        """
        獲取電機的極限開關數值

        Args:
            motorID (int): 電機ID

        Raises:
            ValueError: [description]

        Returns:
            tuple[bool, bool]: 高極限數值, 低極限數值
        """
        if motorID not in range(0, 4):
            raise ValueError("Incorrect Motor ID.")
        canReceiver = self.__lswCanRecv
        _, _, _, data, _ = canReceiver.readMessage()
        if motorID == 0:
            high = data[0]
            low = data[1]
        elif motorID == 1:
            high = data[2]
            low = data[3]
        elif motorID == 2:
            high = data[4]
            low = data[5]
        elif motorID == 3:
            high = data[6]
            low = data[7]
        high = bool(high)
        low = bool(low)
        return high, low

    def getID(self) -> int:
        """
        獲取Titan Quad ID

        Returns:
            int: Titan Quad ID
        """
        return self.__titanID

    def __getMsgID(self, messageType: __MessageType) -> int:
        """
        獲取對應Message Type的完整Message ID

        Args:
            messageType (__MessageType): 模式

        Returns:
            int: 完整Message ID
        """
        return messageType.value + self.__titanMsgID

    @staticmethod
    def __getTitanIDToMsgID(titanID: int) -> int:
        """
        獲取Titan ID轉Titan Message ID

        Args:
            titanID (int): Titan Quad ID

        Returns:
            int: Titan Quad Message ID
        """
        return titanID - 32