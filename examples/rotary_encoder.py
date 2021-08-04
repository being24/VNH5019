#!/usr/bin/env python3

import atexit
from datetime import datetime, timedelta

import pigpio


class RotaryEncoder:
    """Class to decode mechanical rotary encoder pulses."""

    def __init__(self, pi, gpioA, gpioB, callback):
        """
        Instantiate the class with the pi and gpios connected to
        rotary encoder contacts A and B.  The common contact
        should be connected to ground.  The callback is
        called when the rotary encoder is turned.  It takes
        one parameter which is +1 for clockwise and -1 for
        counterclockwise.

        EXAMPLE

        import time
        import pigpio

        import rotary_encoder

        pos = 0

        def callback(way):

           global pos

           pos += way

           print("pos={}".format(pos))

        pi = pigpio.pi()

        decoder = rotary_encoder.decoder(pi, 7, 8, callback)

        time.sleep(300)

        decoder.cancel()

        pi.stop()

        """

        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB
        self.callback = callback

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.pi.set_mode(gpioA, pigpio.INPUT)
        self.pi.set_mode(gpioB, pigpio.INPUT)

        self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)


    def _pulse(self, gpio, level, tick):
        """
        Decode the rotary encoder pulse.

                     +---------+         +---------+      0
                     |         |         |         |
           A         |         |         |         |
                     |         |         |         |
           +---------+         +---------+         +----- 1

               +---------+         +---------+            0
               |         |         |         |
           B   |         |         |         |
               |         |         |         |
           ----+         +---------+         +---------+  1
        """

        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio:  # debounce
            self.lastGpio = gpio

            if gpio == self.gpioA and level == 1:
                if self.levB == 1:
                    self.callback(1)
            elif gpio == self.gpioB and level == 1:
                if self.levA == 1:
                    self.callback(-1)

    def cancel(self):
        """
        Cancel the rotary encoder decoder.
        """

        self.cbA.cancel()
        self.cbB.cancel()
        self.pi.stop()


if __name__ == "__main__":

    import time

    count = 0.0
    prev_current_time = datetime.now()

    # 一カウントあたりの回転角 = 360 / 一回転あたりのパルス / ギア比（１ﾃｲﾊﾞｲ）
    # 立ち上がり立ち下がりが4回
    # https://jp.cuidevices.com/blog/what-is-encoder-ppr-cpr-and-lpr#cpr
    one_count = 360 * 4 / (64 * 50)

    def callback(way):

        global count
        global prev_current_time

        current_time = datetime.now()

        count += way

        pos = count * one_count
        eplased_time = (current_time - prev_current_time) / \
            timedelta(seconds=1)
        rotation_speed = one_count / eplased_time

        print(f"pos={pos:.0f}")
        print(f"rotation_speed={rotation_speed:.0f}")

        prev_current_time = current_time

    pi = pigpio.pi()

    mydecoder = RotaryEncoder(pi, 6, 5, callback)

    time.sleep(300)
