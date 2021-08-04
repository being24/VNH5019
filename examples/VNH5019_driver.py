# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import atexit
from datetime import datetime, timedelta
from rotary_encoder import RotaryEncoder
from logging import DEBUG, INFO, WARN, Formatter, StreamHandler, getLogger

import pigpio
from PCA9685_wrapper import PWM


class VNH5019:
    def __init__(
            self,
            pi: pigpio.pi,
            driver_in1: int,
            driver_in2: int,
            pwm_channel: int,
            logging_level: int = INFO):
        """TA7291Pのインスタンスを初期化

        Args:
            pi (pigpio.pi): pigpioインスタンス
            driver_in1 (int): モータードライバのIN1
            driver_in2 (int): モータードライバのIN2
            pwm_channel (int): PCA9685のチャンネル
            logging_level (int): ロガーのレベル
        """

        # PWMドライバの初期化
        self.pwm = PWM(pwm_channel, freq=1000)

        # pigpioの初期化
        self.pi = pi

        # pin設定をクラス内変数化
        self.in1 = driver_in1
        self.in2 = driver_in2

        # 制御ピンの初期化
        self.pi.set_mode(self.in1, pigpio.OUTPUT)
        self.pi.set_mode(self.in2, pigpio.OUTPUT)
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)

        # loggerの設定
        self.init_logger(logging_level)

        # ロータリーエンコーダ用変数
        self.count = 0.0
        self.prev_current_time = datetime.now()
        self.one_count = 360 * 4 / (64 * 50)
        self.rotation_speed = 0.0
        self.print_counter = 0
        self.angle = 0.0
        self.prev_angle = 0.0
        self.diff = 0.0

        # PID制御用変数
        self.P_gain = 0.5

        # ロータリーエンコーダの初期化
        RotaryEncoder(pi, 6, 5, self.callback)

        # 終了時に全出力を切る
        atexit.register(self.cleanup)

    def init_logger(self, level):
        """ロガーの初期化

        Args:
            level (loggingのレベル): INFO,WARN等で指定
        """

        # ロガーオブジェクト
        self.logger = getLogger(__name__)
        # ログが複数回表示されるのを防止
        self.logger.propagate = False
        # ロガー自体のロギングレベル
        self.logger.setLevel(level)

        # ログを標準出力へ
        handler = StreamHandler()
        # ロギングのフォーマット
        handler.setFormatter(
            Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
        # このハンドラのロギングレベル
        handler.setLevel(DEBUG)
        # ロガーにハンドラを追加
        self.logger.addHandler(handler)

    def callback(self, way):
        current_time = datetime.now()

        self.count += way

        self.angle = self.count * self.one_count

        eplased_angle = self.angle - self.prev_angle
        eplased_time = (current_time - self.prev_current_time) / \
            timedelta(seconds=1)
        self.rotation_speed = eplased_angle / eplased_time
        # print(f"eplased_time={eplased_time:.3f}")

        # self.logger.debug(f"angle={self.angle:.0f}")
        # self.logger.debug(f"rotation_speed={self.rotation_speed:.0f}")

        self.prev_current_time = current_time
        self.prev_angle = self.angle

    def cleanup(self):
        """インスタンス破棄時に実行、出力を止める
        """
        self.free()
        self.pi.stop()

    def free(self):
        """フリー状態にする
        """
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)
        self.pwm.setPWM(0)

    def brake(self):
        """ブレーキ状態にする
        """
        self.pi.write(self.in1, 1)
        self.pi.write(self.in2, 1)
        self.pwm.setPWM(0)

    def drive(self, pwm_duty_cycle: int):
        """モーターを駆動する変数

        Args:
            pwm_duty_cycle (int): int型で設定、エラーハンドリングはライブラリに任せる
        """
        # 正の値であれば正転に設定
        if pwm_duty_cycle > 0:
            self._forward(pwm_duty_cycle)

        # 負の値であれば逆転に設定
        elif pwm_duty_cycle < 0:
            self._back(pwm_duty_cycle)

        # 0ならフリー状態にする
        else:
            self.free()

    def drive_until(self, pwm_duty_cycle: int, drive_time: float):
        """一定の秒数モーターを駆動する変数

        Args:
            pwm_duty_cycle (int): int型で設定、エラーハンドリングはライブラリに任せる
            drive_time (float): 駆動秒数(単位は秒)
        """

        drive_time = drive_time * 1000 * 1000

        start_time = datetime.now()
        while True:
            driven_time = (datetime.now() - start_time) / \
                timedelta(microseconds=1)
            self.drive(pwm_duty_cycle)

            if driven_time > drive_time:
                break

    def _forward(self, pwm_duty_cycle: int):
        """正転用の関数

        Args:
            pwm_duty_cycle (int): duty_cycle
        """
        self.pi.write(self.in1, 1)
        self.pi.write(self.in2, 0)
        self.pwm.setPWM(abs(pwm_duty_cycle))

    def _back(self, pwm_duty_cycle: int):
        """逆転用の関数

        Args:
            pwm_duty_cycle (int): duty_cycle
        """
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 1)
        self.pwm.setPWM(abs(pwm_duty_cycle))

    def get_current_angle(self) -> float:
        """現在の角度を返す

        Returns:
            float: 今の角度
        """
        return self.angle

    def get_rotation_speed(self) -> float:
        """現在の回転速度を返す

        Returns:
            float: 今の回転速度
        """
        return self.rotation_speed

    def reset_angle(self):
        """角度を初期化する
        """
        self.angle = 0
        self.count = 0

    def motor_speed(self, speed: float):
        """モーターを特定速度で回転させる関数

        Args:
            speed (float): 回転速度(degree/s)
        """

        duty_cycle = 5 * speed

        duty_cycle = int(duty_cycle)

        if duty_cycle > 4095:
            duty_cycle = 4095
        elif duty_cycle < -4095:
            duty_cycle = -4095

        self.drive(pwm_duty_cycle=duty_cycle)

    def motor_speed_EX(self, speed: float, KP: float = 1.0):
        """モーターを特定速度で回転させる関数、速度フィードバック付き

        Args:
            speed (float): 回転速度(degree/s)
            KP (float, optional): 比例ゲイン. Defaults to 1.0.
        """

        current_speed = self.get_rotation_speed()

        diff = speed - current_speed
        if abs(diff) > 6000:
            pass
        else:
            self.diff = diff

        raw_gain = speed + KP * self.diff / (500 / abs(speed))
        speed = int(raw_gain)

        if self.print_counter % 50 == 0:
            if abs(self.diff) > 0:
                # print(f"gain={(100 / speed):.5f}")
                print(f"diff={self.diff}")
                print(f"input_speed={speed}")
                # print(f"raw_gain={raw_gain}")
                print(f"current_speed={current_speed:.0f}")

        self.print_counter += 1

        self.motor_speed(speed=speed)

    def drive_motor_speed(self, speed: float, drive_time: float):
        """特定速度で特定時間回転させる関数

        Args:
            speed (float): 回転速度
            drive_time (float): 回転時間
        """
        drive_time = drive_time * 1000 * 1000

        start_time = datetime.now()

        self.print_counter = 0

        while True:
            driven_time = (datetime.now() - start_time) / \
                timedelta(microseconds=1)

            """
            if self.print_counter % 50 == 0:
                current_speed = self.get_rotation_speed()

                # print(f"duty_cycle={duty_cycle}")
                print(f"speed={current_speed:.0f}")
            """

            self.print_counter += 1

            self.motor_speed(speed=speed)

            if driven_time > drive_time:
                self.brake()
                break

    def drive_motor_speed_EX(
            self,
            speed: float,
            drive_time: float,
            KP: float = 1.0):
        """特定速度で特定時間回転させる関数、フィードバック付き

        Args:
            speed (float): [description]
            drive_time (float): [description]
            KP (float, optional): [description]. Defaults to 2.0.
        """
        drive_time = drive_time * 1000 * 1000

        start_time = datetime.now()

        self.print_counter = 0

        while True:
            driven_time = (datetime.now() - start_time) / \
                timedelta(microseconds=1)

            self.motor_speed_EX(speed=speed, KP=KP)

            if driven_time > drive_time:
                self.brake()
                break

    def rotate_motor(self, pwm_duty_cycle: int, rotation_angle: float):
        """モーターを指定角度まで指定のduty比で回転させる関数

        Args:
            pwm_duty_cycle (int): duty比
            rotation_angle (float): 目標角度
        """

        while True:
            current_angle = self.get_current_angle()

            diff = rotation_angle - current_angle

            if diff > 0:
                self._forward(pwm_duty_cycle)
            else:
                self._back(pwm_duty_cycle)

            self.logger.debug(current_angle)

            if abs(diff) < self.one_count:
                self.free()
                break

        cnt = 0
        while True:
            current_angle = self.get_current_angle()

            diff = rotation_angle - current_angle

            if diff > 0:
                self._forward(600)
            else:
                self._back(600)

            self.logger.debug(current_angle)

            if abs(diff) < self.one_count:
                cnt += 1
                if cnt == 10:
                    self.brake()
                    break

    def rotate_motor_EX(
        self,
        rotation_angle: float,
        KP: float = 5,
        max_speed: int = 4095,
        min_speed: int = 800,
    ):
        """モーターをP制御で指定角度まで回転させる関数

        Args:
            rotation_angle (float): 目標角度
            P (float): PIDのP
            max_speed (int): 最大速度
            min_speed (int): 最小速度
        """

        cnt = 0
        if KP == 0:
            raise ValueError("can't define KP as 0.0")

        while True:
            current_angle = self.get_current_angle()

            diff = rotation_angle - current_angle

            pow = KP * diff
            pow = int(pow)
            if abs(pow) < min_speed:
                if pow > 0:
                    pow = min_speed
                else:
                    pow = min_speed * -1
            elif abs(pow) > max_speed:
                if pow > 0:
                    pow = max_speed
                else:
                    pow = max_speed * -1

            # self.drive(pwm_duty_cycle=pow)
            self.motor_speed_EX(speed=pow / 6)

            self.logger.debug(f"current_angle={current_angle}")
            self.logger.debug(f"pow={pow}")

            if abs(diff) <= self.one_count:
                cnt += 1
                if cnt == 10:
                    self.brake()
                    break
