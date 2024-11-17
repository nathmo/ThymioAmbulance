import numpy as np
from threading import Lock
from tdmclient import ClientAsync, aw, thymio
import time

'''
This is the list of property avaible on the node oject. some are function, some are variable. None are documented in a
clear and consistend maneer. Good luck...

print(dir(self.node))
['__class__', 
'__contains__', 
'__delattr__', 
'__dict__', 
'__dir__', 
'__doc__', 
'__enter__', 
'__eq__', 
'__exit__', 
'__format__', 
'__ge__', 
'__getattribute__', 
'__getitem__', 
'__gt__', 
'__hash__', 
'__init__', 
'__init_subclass__', 
'__le__', 
'__lt__', 
'__module__', 
'__ne__', 
'__new__', 
'__reduce__', 
'__reduce_ex__', 
'__repr__',
'__setattr__',
'__setitem__', 
'__sizeof__', 
'__str__', 
'__subclasshook__', 
'__weakref__', 
'add_event_received_listener', 
'add_events_received_listener', 
'add_variables_changed_listener', 
'add_vm_state_changed_listener', 
'clear_event_received_listeners', 
'clear_events_received_listeners', 
'clear_variables_changed_listeners', 
'clear_vm_state_changed_listener', 
'compile', 
'create_msg_lock_node', 
'create_msg_program', 
'create_msg_register_events', 
'create_msg_rename_node', 
'create_msg_request_vm_description', 
'create_msg_scratchpad_update', 
'create_msg_send_events', 
'create_msg_set_variables', 
'create_msg_set_vm_execution_state', 
'create_msg_unlock_node', 
'create_msg_watch_node', 
'event_description', 
'filter_out_vm_events', 
'flash', 
'flush', 
'function_description', 
'get_vm_description', 
'id', 
'id_str', 
'lock', 
'lock_node', 
'mark_change', 
'notify_events_received', 
'notify_variables_changed',
'notify_vm_state_changed', 
'on_event_received',
'on_events_received', 
'on_variables_changed', 
'on_vm_state_changed', 
'props', 
'register_events', 
'remove_event_received_listener', 
'remove_events_received_listener', 
'remove_variables_changed_listener', 
'remove_vm_state_changed_listener', 
'rename', 
'run', 
'send_events', 
'send_lock_node', 
'send_program', 
'send_register_events', 
'send_rename_node', 
'send_request_vm_description', 
'send_send_events', 
'send_set_scratchpad', 
'send_set_variables', 
'send_unlock_node',
'set_properties', 
'set_scratchpad', 
'set_variables', 
'set_vm_execution_state', 
'status', 
'stop', 
'thymio', 
'unlock', 
'unwatch', 
'v',                        -> this is the object that contains the cached variable
'var',                      -> return an empty list
'var_description',          -> return the list of every variable  (see further down)
'var_to_send', 
'vm_description', 
'wait_for_variables', 
'watch', 
'watch_flags', 
'watch_node']


print(self.node.props)  # 
{'node_id': b'O\xef\xd1\xe0\x979D\xcc\xb11\x04\x01\xb8#T\xab',
'node_id_str': '4fefd1e0-9739-44cc-b131-0401b82354ab', 
'group_id': b'\xdf$\xec\x96\x17\xafB"\x9f\xd1c\xa9j4~Q', 
'group_id_str': 'df24ec96-17af-4222-9fd1-63a96a347e51', 
'status': 2, 
'type': -1, 
'name': 'Thymio-II', 
'capabilities': 7, 
'fw_version': '14'}

list of robot variable : 

robot_variables = list(aw(self.node.var_description()))
print(robot_variables)
            
['_id', 
'event.source', 
'event.args', 
'_fwversion', 
'_productId', 
'buttons._raw', 
'button.backward', 
'button.left', 
'button.center', 
'button.forward', 
'button.right', 
'buttons._mean', 
'buttons._noise', 
'prox.horizontal', 
'prox.comm.rx._payloads', 
'prox.comm.rx._intensities', 
'prox.comm.rx', 
'prox.comm.tx', 
'prox.ground.ambiant', 
'prox.ground.reflected', 
'prox.ground.delta', 
'motor.left.target',     -> Theses two are the only properties that can be set to change the speed
'motor.right.target',    -> Theses two are the only properties that can be set to change the speed
'_vbat', 
'_imot', 
'motor.left.speed', 
'motor.right.speed', 
'motor.left.pwm', 
'motor.right.pwm', 
'_integrator', 
'acc', 
'leds.top', 
'leds.bottom.left', 
'leds.bottom.right', 
'leds.circle', 
'temperature',          -> the unit. it increase when hotter. 309 at ambient temp, 328 after blowing hot humid air inside
'rc5.address', 
'rc5.command', 
'mic.intensity', 
'mic.threshold', 
'mic._mean', 
'timer.period', 
'acc._tap', 
'sd.present']

theses value can be modified using :

BEWARE : IT IS NOT RESPONSIVE. EXPECTE 1 sec to apply the command.

aw(self.node.lock())  # Acquire lock explicitly
aw(self.client.wait_for_status(self.client.NODE_STATUS_READY))
v = {
    "motor.left.target": [speed],
    "motor.right.target": [speed],
}
aw(self.node.set_variables(v))
aw(self.node.unlock())  # Release lock
                       
To read the value : 

self.node.lock()
self.client.aw(self.node.wait_for_variables())
temperature = self.node.v.temperature
self.node.unlock() 

'''

class RobotMovement:
    def __init__(self):
        # Connect to the robot using tdmclient
        self.client = ClientAsync()
        self.node = None
        self.waypoints = [np.array([0, 0, 0])]  # List of np.array for waypoints
        self.position = np.array([0, 0, 0])  # Current position of the robot
        self.speed = np.array([0, 0, 0])  # Current speed of the robot
        self._lock = Lock()  # Mutex lock for thread safety

    def __del__(self):
        # Ensure the node is unlocked before the instance is destroyed
        if self.node is not None:
            try:
                aw(self.node.unlock())
                print("Robot unlocked successfully.")
            except Exception as e:
                print(f"Error unlocking the robot: {e}")

    def connect(self):
        self.node = aw(self.client.wait_for_node())

    def set_waypoints(self, waypoints):
        with self._lock:
            self.waypoints = waypoints

    def get_waypoints(self):
        with self._lock:
            return self.waypoints

    def get_position(self):
        with self._lock:
            return self.position

    def set_position(self, kalman_position):
        with self._lock:
            self.position = kalman_position

    def get_speed(self):
        with self._lock:
            return self.speed

    def update(self):
        with self._lock:
            # Placeholder for movement and obstacle avoidance logic
            pass

    def mm_per_second_to_arbitrary_unit(self, v, k=0.39, c=0.409):
        """
        Convert speed in mm/s to the arbitrary motor unit using a linear model.
        :param v: Speed in mm/s
        :param k: Slope (conversion factor) in mm/s per arbitrary unit (default: 0.39)
        :param c: Offset (constant) in mm/s (default: 0.409)
        :return: Speed in arbitrary motor units


        44 mm in 10 sec @ speed = 10
        80 mm in 10 sec @ speed = 20
        120 mm in 10 sec @ speed = 30
        155 mm in 10 sec @ speed = 40
        200 mm in 10 sec @ speed = 50

        230 mm in 10 sec @ speed = 60
        260 mm in 10 sec @ speed = 70
        290 mm in 10 sec @ speed = 80
        330 mm in 10 sec @ speed = 90
        365 mm in 10 sec @ speed = 100

-
        88 mm in 20 sec @ speed = 10
        160 mm in 20 sec @ speed = 20
        228 mm in 20 sec @ speed = 30
        305 mm in 20 sec @ speed = 40
        385 mm in 20 sec @ speed = 50

        88 mm in 20 sec @ speed = 10 LOADED (1kg)

        From these data we find 0.39 mm/s for each arbitrary speed unit + 0.409 mm/s (offset) (valid in the 10-100 range)
        """
        # Ensure the speed is greater than the offset for valid conversion, otherwise set the speed to 0
        if abs(v) < c:
            return 0

        # Perform the conversion
        S = (abs(v) - c) / k
        return int(S*(1 if v >= 0 else -1))

    def set_speed(self, speed_left, speed_right):
        """
        Set the speed of each wheel of the robot.
        :param speed: Integer speed value (in mm/s) [-100:-10] OR [10:100] Negative values move backward for each motor.
        it might work outside theses value but you will see that the conversion is not as precise.
        """

        if self.node is not None:
            aw(self.node.lock())  # Acquire lock explicitly
            aw(self.client.wait_for_status(self.client.NODE_STATUS_READY))
            v = {
                "motor.left.target": [self.mm_per_second_to_arbitrary_unit(speed_left)],
                "motor.right.target": [self.mm_per_second_to_arbitrary_unit(speed_right)],
            }
            aw(self.node.set_variables(v))
            robot_variables = self.node.v
            print(robot_variables)
            aw(self.node.unlock())  # Release lock
        else:
            print("robot not connected")

    def set_straight_speed(self, speed):
        """
        Set the speed of the robot.
        :param speed: Integer speed value (in mm/s) [-100:-10] OR [10:100] Negative values move backward.
        it might work outside theses value but you will see that the conversion is not as precise.
        """
        if self.node is not None:
            aw(self.node.lock())  # Acquire lock explicitly
            aw(self.client.wait_for_status(self.client.NODE_STATUS_READY))
            v = {
                "motor.left.target": [self.mm_per_second_to_arbitrary_unit(speed)],
                "motor.right.target": [self.mm_per_second_to_arbitrary_unit(speed)],
            }
            aw(self.node.set_variables(v))
            robot_variables = self.node.v
            print(robot_variables)
            aw(self.node.unlock())  # Release lock
        else:
            print("robot not connected")

    def set_angular_speed(self, angular_speed):
        """
        Turn the robot on the spot by a given angle.
        :param angle: Angle in radians/per second (positive = clockwise rotation).

        the wheel are spaced 95 mm from each other, 47.5mm radius
        for a full turn, each wheel travel along a perimeter of 298.45 mm
        assuming max speed of 100 mm/s for precision, thats 1.5 sec for a full 180° /pi radian
        """
        motor_speed = self.mm_per_second_to_arbitrary_unit(angular_speed*298.45/(2*np.pi))
        if self.node is not None:
            self.node.lock()
            v = {
                "motor.left.target": [motor_speed],
                "motor.right.target": [-motor_speed],
            }
            aw(self.node.set_variables(v))
            self.node.unlock()
        else:
            print("robot not connected")

    def get_temperature(self):
        """
        return the temperature in an arbitrary unit (or in kelvin and really off by 10-20 K)
        """
        if self.node is not None:
            self.node.lock()
            self.client.aw(self.node.wait_for_variables())
            temperature = self.node.v.temperature
            self.node.unlock()
            return temperature
        else:
            print("robot not connected")

    def get_all_variable(self):
        if self.node is not None:
            self.node.lock()
            self.client.aw(self.node.wait_for_variables())
            # Get all attributes of node.v
            variables = dir(self.node.v)

            # Filter out private attributes and methods
            variable_names = list(aw(self.node.var_description()))

            # Retrieve each variable's value
            variable_values = {var: getattr(self.node.v, var) for var in variable_names}
            self.node.unlock()
            return variable_values
        else:
            print("robot not connected")


if __name__ == "__main__":
    robot = RobotMovement()
    robot.connect()
    print(robot.get_all_variable())
    #robot.turn_angle(np.pi)
    print("start set speed to 50")
    robot.set_straight_speed(90)
    print("done set speed to 10")
    #robot.turn_angle(np.pi)
    time.sleep(10)
    print("start set speed to 0")
    robot.set_straight_speed(0)
    print("done set speed to 0")
