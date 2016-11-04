#!/usr/bin/env python

from __future__ import print_function

import time
import logging
import argparse
import collections
import dynamixel_functions as dxl

log = logging.getLogger('servode')
# logging.basicConfig(datefmt='%(asctime)s - %(name)s:%(levelname)s: %(message)s')
handler = logging.StreamHandler()
formatter = logging.Formatter(
    '%(asctime)s|%(name)-8s|%(levelname)s: %(message)s')
handler.setFormatter(formatter)
log.addHandler(handler)
log.setLevel(logging.INFO)

# Protocol version
PROTOCOL_V = 1  # Set protocol version used with the Dynamixel AX-12

# Default setting
BAUDRATE_PERM = 1000000
BAUDRATE_TEMP = 500000
TRUE = 1
FALSE = 0
ON = 1
OFF = 0

# Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
DEVICENAME = "/dev/ttyUSB0".encode('utf-8')

COMM_SUCCESS = 0  # Communication Success result value
COMM_TX_FAIL = -1001  # Communication Tx Failed

# Dynamixel control table addresses
dxl_control = {
    "model_number": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 0,
        "comm_bytes": 2,
        "access": "r"
    },
    "firmware_version": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 2,
        "comm_bytes": 1,
        "access": "r"
    },
    "ID": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 3,
        "comm_bytes": 1,
        "access": "rw"
    },
    "baud_rate": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 4,
        "comm_bytes": 1,
        "access": "rw"
    },
    "return_delay": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 5,
        "comm_bytes": 1,
        "access": "rw"
    },
    "cw_angle_limit": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 6,
        "comm_bytes": 2,
        "access": "rw"
    },
    "ccw_angle_limit": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 8,
        "comm_bytes": 2,
        "access": "rw"
    },
    "highest_limit_temperature": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 11,
        "comm_bytes": 1,
        "access": "rw"
    },
    "lowest_limit_voltage": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 12,
        "comm_bytes": 1,
        "access": "rw"
    },
    "highest_limit_voltage": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 13,
        "comm_bytes": 1,
        "access": "rw"
    },
    "max_torque": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 14,
        "comm_bytes": 2,
        "access": "rw"
    },
    "status_return_level": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 16,
        "comm_bytes": 1,
        "access": "rw"
    },
    "alarm_LED": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 17,
        "comm_bytes": 1,
        "access": "rw"
    },
    "alarm_shutdown": {
        "addr_type": "EEPROM",
        "volatile": False,
        "address": 18,
        "comm_bytes": 1,
        "access": "rw"
    },
    "torque_enable": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 24,
        "comm_bytes": 1,
        "access": "rw"
    },
    "LED": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 25,
        "comm_bytes": 1,
        "access": "rw"
    },
    "cw_compliance_margin": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 26,
        "comm_bytes": 1,
        "access": "rw"
    },
    "ccw_compliance_margin": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 27,
        "comm_bytes": 1,
        "access": "rw"
    },
    "cw_compliance_slope": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 28,
        "comm_bytes": 1,
        "access": "rw"
    },
    "ccw_compliance_slope": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 29,
        "comm_bytes": 1,
        "access": "rw"
    },
    "goal_position": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 30,
        "comm_bytes": 2,
        "access": "rw"
    },
    "moving_speed": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 32,
        "comm_bytes": 2,
        "access": "rw"
    },
    "torque_limit": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 34,
        "comm_bytes": 2,
        "access": "rw"
    },
    "present_position": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 36,
        "comm_bytes": 2,
        "access": "r"
    },
    "present_speed": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 38,
        "comm_bytes": 2,
        "access": "r"
    },
    "present_load": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 40,
        "comm_bytes": 2,
        "access": "r"
    },
    "present_voltage": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 42,
        "comm_bytes": 1,
        "access": "r"
    },
    "present_temperature": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 43,
        "comm_bytes": 1,
        "access": "r"
    },
    "registered_instruction": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 44,
        "comm_bytes": 1,
        "access": "r"
    },
    "moving": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 46,
        "comm_bytes": 1,
        "access": "r"
    },
    "lock": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 47,
        "comm_bytes": 1,
        "access": "rw"
    },
    "punch": {
        "addr_type": "RAM",
        "volatile": True,
        "address": 48,
        "comm_bytes": 2,
        "access": "rw"
    }
}


class Servo(object):
    def __init__(self, sp, servo_id=1, read_cache=None):
        """

        :param sp: the ServoProtocol to use with this Servo
        :param servo_id: the ID of the servo on the servo protocol chain
        :param read_cache: a cache in which the latest values read from a
           register will be placed. Each value is stored with key 'register'.
        """
        super(Servo, self).__init__()
        self.wheel = False
        self.enable_torque = True
        self.servo_id = servo_id
        self.sp = sp
        self.read_cache = read_cache
        log.debug("[Servo.__init__] read_cache:{0}".format(read_cache))

    def wheel_mode(self, enable=True):
        if enable:
            self.sp.write_register(self.servo_id, "cw_angle_limit", 0)
            self.sp.write_register(self.servo_id, "ccw_angle_limit", 0)
            log.info("[wheel_mode] wrote enable=True registers")
        else:
            self.sp.write_register(self.servo_id, "cw_angle_limit", 4)
            self.sp.write_register(self.servo_id, "ccw_angle_limit", 42)
            log.info("[wheel_mode] wrote enable=False registers")

    def wheel_speed(self, speed=512, cw=True):
        """
        Set the Servo's wheel speed. If the servo is not in wheel mode it will
        be placed into wheel mode.

        :param speed: value between 0-1023
        :param cw: True > clockwise or False > counter-clockwise
        :return:
        """
        if (0 <= speed <= 1023) is False:
            raise ValueError("Invalid speed value:{0}".format(speed))

        self.wheel_mode()

        set_speed = speed
        if cw is False:
            set_speed = 1024 + speed

        self.sp.write_register(self.servo_id, "moving_speed", set_speed)
        log.info("[wheel_speed] wrote speed value:{0}".format(set_speed))

    def new_id(self, new_id):
        if (0 <= new_id <= 252) is False:
            raise ValueError("Invalid new_id value:{0}".format(new_id))
        self.sp.write_register(self.servo_id, "ID", new_id)
        log.info("[new_id] servo_id:{0} given new_id value:{1}".format(
            self.servo_id, new_id))
        self.servo_id = new_id

    def read(self, register):
        value = self.sp.read_register(self.servo_id, register)
        if self.read_cache is not None:
            self.read_cache[register] = value
        return value

    def write(self, register, value):
        return self.sp.write_register(self.servo_id, register, value)

    def __getitem__(self, name):
        return self.read(name)

    def __setitem__(self, key, val):
        self.write(key, val)


class ServoGroup(object):
    """
    A Group of Servos that will remain in order while interacting with or
    iterating over them.
    """
    def __init__(self):
        super(ServoGroup, self).__init__()
        self.servos = collections.OrderedDict()

    def __len__(self):
        return len(self.servos)

    def __getitem__(self, name):
        return self.servos[name]

    def __setitem__(self, key, val):
        self.servos[key] = val

    def __iter__(self):
        return iter(self.servos)

    def __repr__(self):
        dictrepr = self.servos.__repr__(self)
        return '%s(%s)'.format((type(self).__name__, dictrepr))

    def write(self, register, values):
        """
        Write the list of values to the register on every servo in the
        ServoGroup.
        Note: the length of the values list should equal the length of the
        ServoGroup

        :param register:
        :param values:
        :return:
        """
        t = 0
        log.debug('[ServoGroup.write] len(self):{0} len(values):{1}'.format(
            len(self), len(values)))

        for servo in self.servos.values():
            if t < min(len(self), len(values)):
                log.info("[ServoGroup.write] servo:{0} values[{1}]:{2}".format(
                    servo, t, values[t]
                ))
                servo.write(register, values[t])
            else:
                log.warn("[ServoGroup.write] more group members than values.")
            t += 1


class ServoProtocol(object):
    """
    A Pythonic implementation of a ServoProtocol.
    """
    def __init__(self, baud_rate=BAUDRATE_PERM, manufacturer='ROBOTIS',
                 servo_type='AX-12'):
        """

        :param baud_rate:
        :param manufacturer:
        :param servo_type:
        """
        super(ServoProtocol, self).__init__()
        if servo_type == 'AX-12':
            self.servo_type = servo_type
        else:
            raise NotImplementedError("servo_type:{0} not understood.".format(
                servo_type))
        self.baud_rate = baud_rate
        self.manufacturer = manufacturer
        self.port_num = dxl.portHandler(DEVICENAME)
        dxl.packetHandler()  # Initialize PacketHandler Structs

    def __enter__(self):
        log.debug("[ServoProtocol.__enter__] Connection information")

        # Open port
        if dxl.openPort(self.port_num):
            log.debug("[ServoProtocol.__enter__] opened port:{0}".format(
                self.port_num))
        else:
            raise IOError("[ServoProtocol.__enter__] Failed to open the port!")

        # Set port baudrate to PERM
        if dxl.setBaudRate(self.port_num, self.baud_rate):
            log.debug("[ServoProtocol.__enter__] Set baud rate to: {0}".format(
                self.baud_rate))
        else:
            log.debug(
                "[ServoProtocol.__enter__] Failed to change the baud rate!")
            return

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        log.debug("[ServoProtocol.__exit__] closing dxl port")
        dxl.closePort(self.port_num)

    def factory_reset(self, servo):
        """

        :param servo: the servo or servo id to have a factory reset
        :return: None
        """
        if isinstance(servo, Servo):
            sid = servo.servo_id
        else:
            sid = servo

        log.debug("[factory_reset] Try reset:{0}".format(sid))
        dxl.factoryReset(self.port_num, PROTOCOL_V, sid, 0x00)
        if dxl.getLastTxRxResult(self.port_num,
                                 PROTOCOL_V) != COMM_SUCCESS:
            log.error("[factory_reset] Aborted")
            dxl.printTxRxResult(
                PROTOCOL_V,
                dxl.getLastTxRxResult(self.port_num, PROTOCOL_V))
            return
        elif dxl.getLastRxPacketError(self.port_num, PROTOCOL_V) != 0:
            dxl.printRxPacketError(
                PROTOCOL_V,
                dxl.getLastRxPacketError(self.port_num, PROTOCOL_V))

        # Wait for reset
        log.debug("[factory_reset] Wait for reset...")
        time.sleep(2)
        log.debug("[factory_reset] Reset complete.")

    def ping(self, servo):
        """

        :param servo: the servo or servo id to be pinged
        :return: None
        """
        if isinstance(servo, Servo):
            sid = servo.servo_id
        else:
            sid = servo

        dxl_model_number = dxl.pingGetModelNum(
            self.port_num, PROTOCOL_V, sid)

        if dxl.getLastTxRxResult(self.port_num,
                                 PROTOCOL_V) != COMM_SUCCESS:
            dxl.printTxRxResult(
                PROTOCOL_V,
                dxl.getLastTxRxResult(self.port_num, PROTOCOL_V))
        elif dxl.getLastRxPacketError(self.port_num, PROTOCOL_V) != 0:
            dxl.printRxPacketError(
                PROTOCOL_V,
                dxl.getLastRxPacketError(self.port_num, PROTOCOL_V))
        return dxl_model_number

    def read_register(self, servo, register):
        """

        :param servo: a Servo object or an integer servo_id
        :param register: the register from which to read a value
        :return: the value read from the register
        """
        value = ''
        if isinstance(servo, Servo):
            sid = servo.servo_id
        else:
            sid = servo

        if dxl_control[register]['comm_bytes'] == 1:
            value = dxl.read1ByteTxRx(
                self.port_num, PROTOCOL_V, sid,
                dxl_control[register]['address']
            )
        elif dxl_control[register]['comm_bytes'] == 2:
            value = dxl.read2ByteTxRx(
                self.port_num, PROTOCOL_V, sid,
                dxl_control[register]['address']
            )

        last_result = dxl.getLastTxRxResult(
            self.port_num, PROTOCOL_V
        )
        if last_result != COMM_SUCCESS:
            dxl.printTxRxResult(
                PROTOCOL_V,
                dxl.getLastTxRxResult(self.port_num, PROTOCOL_V)
            )
            log.error("Communication unsuccessful:{0}".format(last_result))
        elif dxl.getLastRxPacketError(self.port_num, PROTOCOL_V) != 0:
            dxl.printRxPacketError(
                PROTOCOL_V,
                dxl.getLastRxPacketError(self.port_num, PROTOCOL_V)
            )
            log.error("Unknown error:{0}".format(last_result))

        return value

    def write_register(self, servo, register, value):
        """

        :param servo: a Servo object or an integer servo_id
        :param register: the register from which to read a value
        :param value: the value to write to the register
        :return: True = success | False = failure
        """
        if isinstance(servo, Servo):
            sid = servo.servo_id
        else:
            sid = servo

        log.debug("[write_register] servo id:{0} reg:'{1}' reg_addr:{2}".format(
            sid, register, dxl_control[register]['address']))

        if dxl_control[register]['access'] == "r":
            raise IOError("register:'{0}' cannot be written".format(register))

        if dxl_control[register]['comm_bytes'] == 1:
            dxl.write1ByteTxRx(
                self.port_num, PROTOCOL_V, sid,
                dxl_control[register]['address'], value)
        elif dxl_control[register]['comm_bytes'] == 2:
            dxl.write2ByteTxRx(
                self.port_num, PROTOCOL_V, sid,
                dxl_control[register]['address'], value)

        last_result = dxl.getLastTxRxResult(
            self.port_num, PROTOCOL_V
        )
        if last_result != COMM_SUCCESS:
            dxl.printTxRxResult(
                PROTOCOL_V,
                dxl.getLastTxRxResult(self.port_num, PROTOCOL_V)
            )
            return False
        elif dxl.getLastRxPacketError(self.port_num, PROTOCOL_V) != 0:
            dxl.printRxPacketError(
                PROTOCOL_V,
                dxl.getLastRxPacketError(self.port_num, PROTOCOL_V)
            )
            return False
        else:
            log.debug(
                "[write_register] register:'{0}' written".format(register))

        return True


def read_all_servo_registers(cli, servo_type='AX-12'):
    with ServoProtocol(servo_type=servo_type) as sp:
        for register in sorted(dxl_control):
            value = sp.read_register(cli.servo_id, register)
            log.info("Registry entry:'{0}' has value: {1}".format(
                register, value))


def wheel_test(cli):
    with ServoProtocol() as sp:
        s = Servo(sp, servo_id=cli.servo_id)
        s.wheel_mode()
        s.wheel_speed(512)
        time.sleep(15)
        s.wheel_speed(512, cw=False)
        time.sleep(15)
        s.wheel_speed(0)


def blink_led(cli):
    with ServoProtocol() as sp:
        i = 0
        while i < 15:
            sp.write_register(cli.servo_id, "LED", ON)
            time.sleep(1)
            sp.write_register(cli.servo_id, "LED", OFF)
            time.sleep(1)
            i += 1


def read_register(cli):
    log.info("Read register: '{0}'".format(cli.register))
    with ServoProtocol() as sp:
        if cli.sid is not None:
            for sid in cli.sid:
                # example use of the Servo class to read a register
                s = Servo(sp=sp, servo_id=sid)
                value = s[cli.register]
                log.info("Servo:{0} value:{1}".format(sid, value))
        else:
            log.info("Servo 'default'")
            # example use of the ServoProtocol to read a register
            value = sp.read_register(servo=1, register=cli.register)
            log.info("Servo:{0} value:{1}".format(1, value))


def write_register(cli):
    with ServoProtocol() as sp:
        if cli.sid is not None:
            for sid in cli.sid:
                sp.write_register(
                    servo=sid,
                    register=cli.register,
                    value=cli.value
                )
                log.info("Servo:{0} wrote value:{1} to register:{2}".format(
                    sid, cli.value, cli.register))
        else:
            log.info("Servo 'default'")
            value = sp.write_register(
                servo=1,
                register=cli.register,
                value=cli.value
            )
            log.info("Servo:{0} wrote value:{1} to register:{2}".format(
                1, value, cli.register))


def to_goal(cli):
    with ServoProtocol() as sp:
        if cli.sg is not None:
            for servo_goal in cli.sg:
                log.info('Servo goal:{0}'.format(servo_goal))
                sp.write_register(
                    servo_goal[0], 'goal_position', servo_goal[1])
        else:
            log.info("Servo: 1 goal: 0")
            sp.write_register(1, 'goal_position', 0)


def factory_reset(cli):
    with ServoProtocol() as sp:
        sp.factory_reset(servo=cli.servo_id)


def change_id(cli):
    with ServoProtocol() as sp:
        s = Servo(sp, servo_id=cli.servo_id)
        s.new_id(cli.new_id)


def ping(cli):
    with ServoProtocol() as sp:
        pong = sp.ping(servo=cli.servo_id)
        log.info("Ping result, model_number:{0}".format(pong))


def torque_enable(cli):
    with ServoProtocol() as sp:
        if cli.torque:
            sp.write_register(cli.servo_id, 'torque_enable', 1)
        else:
            sp.write_register(cli.servo_id, 'torque_enable', 0)
        log.info("Servo:{0} enable torque:{1}".format(cli.servo_id, cli.torque))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Servo Protocol implementation and some common functions',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--debug', dest='debug', action='store_true',
                        help="Activate debug logging level")

    subparsers = parser.add_subparsers()

    all_registers = subparsers.add_parser(
        'all_registers',
        description='Read all registers from a servo')
    all_registers.add_argument('servo_id', nargs='?', default=1, type=int,
                               help="The servo_id to read registers.")
    all_registers.set_defaults(func=read_all_servo_registers)

    wheel_parser = subparsers.add_parser(
        'wheel_test',
        description='Put the Servo in wheel mode and test comms.')
    wheel_parser.add_argument('servo_id', nargs='?', default=1, type=int,
                              help="The servo_id to use to test wheel mode.")
    wheel_parser.set_defaults(func=wheel_test)

    blink_parser = subparsers.add_parser(
        'blink_led',
        description='Blink the LED of the Servo for 30 seconds.')
    blink_parser.add_argument('servo_id', nargs='?', default=1, type=int,
                              help="The servo_id to use for LED blinking.")
    blink_parser.set_defaults(func=blink_led)

    reset_parser = subparsers.add_parser(
        'factory_reset',
        description='Reset the Servo to factory original settings.')
    reset_parser.add_argument(
        'servo_id', nargs='?', default=1, type=int,
        help="The servo_id to perform a factory reset upon.")
    reset_parser.set_defaults(func=factory_reset)

    change_id_parser = subparsers.add_parser(
        'change_id',
        description='Change the ID of the given Servo to a new ID')
    change_id_parser.add_argument(
        'servo_id', nargs='?', default=1, type=int,
        help="The current servo_id of the Servo to change.")
    change_id_parser.add_argument('new_id', nargs='?', default=1, type=int,
                                  help="The new servo_id.")
    change_id_parser.set_defaults(func=change_id)

    ping_parser = subparsers.add_parser(
        'ping',
        description='Ping the Servo and get model_number.')
    ping_parser.add_argument('servo_id', nargs='?', default=1, type=int,
                             help="The servo_id of the Servo to ping.")
    ping_parser.set_defaults(func=ping)

    torque_enable_parser = subparsers.add_parser(
        'torque_enable',
        description='Set torque enable for the specified servo.'
    )
    torque_enable_parser.add_argument(
        'servo_id', nargs='?', default=1, type=int,
        help='The ID of the Servo to set the torque enable register.')
    torque_enable_parser.add_argument(
        '--enable', dest='torque',
        action='store_true',
        help="Enable toggle_enable")
    torque_enable_parser.add_argument(
        '--disable', dest='torque',
        action='store_false',
        help="Disable torque_enable")
    torque_enable_parser.set_defaults(func=torque_enable, torque=True)

    goal_position_parser = subparsers.add_parser(
        'to_goal', description='Move one or more servos to the goal position.'
    )
    goal_position_parser.add_argument(
        '--sg', nargs=2, type=int, action='append',
        help='Servo ID and goal position. [Ex: --sg <sid> <position>]')
    goal_position_parser.set_defaults(func=to_goal)

    read_register_parser = subparsers.add_parser(
        'read_register',
        description='Read the given register from one or more Servos.')
    read_register_parser.add_argument(
        'register', nargs='?', default='present_position',
        help="The Servo register to read.")
    read_register_parser.add_argument(
        '--sid', action='append', type=int,
        help="A servo_id. [one or more arguments]")
    read_register_parser.set_defaults(func=read_register)

    write_register_parser = subparsers.add_parser(
        'write_register',
        description='Write the value to the register of one or more Servos.')
    write_register_parser.add_argument(
        'register', nargs='?', default='goal_position',
        help="The Servo register to write.")
    write_register_parser.add_argument(
        'value', nargs='?', default='0',
        help="The value to write to the register.")
    write_register_parser.add_argument(
        '--sid', action='append', type=int,
        help="A servo_id. [one or more arguments]")
    write_register_parser.set_defaults(func=write_register)

    args = parser.parse_args()
    if args.debug:
        log.setLevel(logging.DEBUG)

    args.func(args)
