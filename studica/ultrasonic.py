from pyMobileRobotics.hal.digital_output import DigitalOutput
from pyMobileRobotics.hal.input_capture import InputCapture


class Ultrasonic():
    """
    超聲波感測器

    支持的感測器型號:
     - HC-SR04
    """

    def __init__(self, trigChannel: int, echChannel: int):
        self.__trigDO = DigitalOutput(trigChannel)
        self.__echoIP = InputCapture(echChannel, counterClockSource=InputCapture.CounterClockSource.INTERNAL,
                 counterDirection=InputCapture.CounterDirection.DIRECTION_UP, slaveMode=InputCapture.SlaveMode.SLAVEMODE_RESET,
                 slaveModeTriggerSource=InputCapture.SlaveModeTriggerSource.TRIGGER_DYNAMIC,
                 captureChannelSource=InputCapture.CaptureChannelSource.CAPTURE_SIGNAL_DYNAMIC,
                 captureChannelPrescaler=InputCapture.CaptureChannelPrescaler.x1,
                 stallAction=InputCapture.StallAction.ACTION_NONE,
                 captureChannelFilterNumSamples=2)

    def pingRAW(self) -> int:
        """
        發射超聲波並讀取反射時間

        Returns:
            int: 反射時間(microseconds)
        """
        self.__trigDO.pulse(True, 10)
        _, raw = self.__echoIP.get()
        return raw

    def pingCM(self, temperature=26) -> float:
        """
        發射超聲波並讀取公分距離

        Args:
            temperature (int, optional): 攝氏溫度. Defaults to 26.

        Returns:
            float: 公分
        """
        raw = self.pingRAW()
        cm = (raw / 2) / (1 / ((331.5 + 0.607 * temperature) * 100 / 1000000))
        return cm

    def pingInch(self, temperature=26) -> float:
        """
        發射超聲波並讀取英吋距離

        Args:
            temperature (int, optional): 攝氏溫度. Defaults to 26.

        Returns:
            float: 英吋
        """
        raw = self.pingRAW()
        inch = (raw / 2) / ((1 / ((331.5 + 0.607 * temperature) * 100 / 1000000)) * 2.54)
        return inch