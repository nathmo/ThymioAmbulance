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
'button.backward',  -> 0, no contact, 1= touched, (int) 
'button.left',      -> 0, no contact, 1= touched, (int) 
'button.center',    -> 0, no contact, 1= touched, (int) 
'button.forward',   -> 0, no contact, 1= touched, (int) 
'button.right',     -> 0, no contact, 1= touched, (int) 
'buttons._mean',  
'buttons._noise', 
'prox.horizontal',     -> a list of 7 value from 0 (nothing dected) to 4096 (very close to obstacle), index 0 is front_left, then, front_center_left, front_center, front_center_right, front_right, rear_left, rear_right (index 6)
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

priority_1 = 1000
priority_2 = 2000
priority_3 = 3000
Kp = 0.5
Kd = 0.05
Ki = 0.05
Kp_angle = 1
forward_speed_max = 150
forward_speed_min = 10
I_distance = 0



class RobotMovement:
    def __init__(self):
        # Connect to the robot using tdmclient
        self.client = ClientAsync()
        self.node = None
        self.waypoints = [np.array([0.0, 0.0, 0.0])]  # List of np.array for waypoints , (a list of numpy array coded like position)
        self.position = np.array([0.0, 0.0, 0.0])  # Current position of the robot (x [mm], y[mm], angle with the x/horizontal axis in radian (clockwise rotation is positive))
        self.speed = np.array([0.0, 0.0, 0.0])  # Current speed of the robot (same format as position but per second)
        self._lock = Lock()  # Mutex lock for thread safety
        self.last_update_time = time.time()
        self.total_elapsed_time = 0
        self.update_count = 0
        self.distance = 0
        self.prev_distance = 0

    def __del__(self):
        # Ensure the node is unlocked before the instance is destroyed
        if self.node is not None:
            try:
                self.set_straight_speed(0)
                aw(self.node.unlock())
                print("Robot unlocked successfully.")
            except Exception as e:
                print(f"Error unlocking the robot: {e}")

    def connect(self):
        print("IP address "+str(self.client.tdm_addr))
        print("Port "+str(self.client.tdm_port))
        self.node = aw(self.client.wait_for_node())
        aw(self.node.lock())
        aw(self.client.wait_for_status(self.client.NODE_STATUS_READY))

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
            self.position = (kalman_position)

    def get_speed(self):
        with self._lock:
            return self.speed

    def set_speed(self, speed):
        with self._lock:
            self.speed = speed
          
 

    def update(self):
        global I_distance
        current_time = time.time()

        elapsed_time = current_time - self.last_update_time
        self.last_update_time = current_time

        # Update average elapsed time
        self.total_elapsed_time += elapsed_time
        self.update_count += 1
        average_elapsed_time = self.total_elapsed_time / self.update_count

        # Increment position by speed * elapsed_time
        position = self.get_position()
        position += self.get_speed() * elapsed_time
        self.set_position(position)

        position[2] = (position[2] +np.pi) % (2 * np.pi) - np.pi  # Normalize angle to [0, 2*pi]

        waypoints = self.get_waypoints()
        
        if waypoints:
            if self.node is not None:
                v = {
                    "leds.top": [0, 0, 32]
                }
                aw(self.node.set_variables(v))

            next_waypoint = waypoints[0]
            delta = next_waypoint - self.position
            self.prev_distance = self.distance
            self.distance = np.linalg.norm(delta[:2])
            
            # Compute signed distance and angular difference
            angle_to_target = np.arctan2(delta[1], delta[0])
            command_angle = angle_to_target - self.position[2]

            if not (int(self.update_count) % max(10, int(1/(min(1, average_elapsed_time))))) :
                print("-------------------------------------------")
                print("speed "+str(self.get_speed()))
                print("position "+str(position))
                print("target " + str(next_waypoint))
                print("Distance to target : "+str(self.distance))
                print("angle to target : " + str(command_angle))
                print("average elapsed time "+str(average_elapsed_time))
                print("time since last print "+str(average_elapsed_time*self.update_count))
            
            # Normalize command_angle to [-pi, pi]
            command_angle = (command_angle + np.pi) % (2 * np.pi) - np.pi

            ir_sensors = self.get_proximity_ir_sensor()
             
            if ir_sensors["front_center"] > priority_2 or ir_sensors["front_center_right"] > priority_1 or ir_sensors["front_right"] > priority_3:
                self.set_angular_speed(-1)
                self.set_speed(np.array([0.0, 0.0, -1.0]))

                # Add forward speed because the direction change is not enough to avoid the obstacle
                self.set_motor_speed(20-1, 20+1)
                self.set_speed(np.array([20.0*np.cos(-1), 20.0*np.sin(-1), 0.0]))                           
                print("Obstacle detected on the right")

            elif ir_sensors["front_center_left"] > priority_1 or ir_sensors["front_left"] > priority_3:
                self.set_angular_speed(1)
                self.set_speed(np.array([0.0, 0.0, 1.0]))

                # Add forward speed because the direction change is not enough to avoid the obstacle
                self.set_motor_speed(20+1, 20-1)
                self.set_speed(np.array([20.0*np.cos(1), 20.0*np.sin(1), 0.0]))                           
                print("Obstacle detected on the left")                

            elif (abs(command_angle) > 0.05):
                min_time = max(3 * average_elapsed_time, 1.0)
                angular_speed = command_angle / min_time
                angular_speed = angular_speed * Kp_angle
                sign = np.sign(angular_speed)
                angular_speed = max(0.5, abs(angular_speed))
                angular_speed = min(4.5, abs(angular_speed))
                angular_speed = round(angular_speed, 1)*sign
                self.set_speed(np.array([0.0, 0.0, angular_speed]))                
                self.set_angular_speed(angular_speed)

            elif (abs(self.distance) > 2.0): # and abs(final_angle) > 0.2: 
                P = Kp * self.distance
                D = Kd * (self.distance - self.prev_distance) / elapsed_time
                I_distance += Ki * (self.distance) * elapsed_time
                if abs(I_distance) > 0.5:
                    I_distance = 0.5 * np.sign(I_distance)
                forward_speed = P + D + I_distance                    
                if abs(forward_speed) > forward_speed_max:
                    forward_speed = forward_speed_max * np.sign(forward_speed)            
                forward_speed = round(forward_speed, 0)
                self.set_speed(np.array([forward_speed*np.cos(angle_to_target), forward_speed*np.sin(angle_to_target), 0.0]))            
                self.set_motor_speed(forward_speed+angle_to_target, forward_speed-angle_to_target)
                

            elif ((abs(self.distance) < 2.0) and (abs(command_angle) < 0.05)):
                # Reached waypoint, update to the next
                self.set_waypoints(waypoints[1:])
                print("Waypoint reached")

            if self.node is not None:
                v = {
                    "leds.top": [32, 0, 0]
                }
                aw(self.node.set_variables(v))
        else:
            self.set_speed(np.array([0.0, 0.0, 0.0]))
            self.set_motor_speed(0.0, 0.0)
            if self.node is not None:
                v = {
                    "leds.top": [0, 0, 0]
                }
                aw(self.node.set_variables(v))
            if not (int(self.update_count) % max(5, int(1 / (min(1, average_elapsed_time))))):
                print("no more waypoints")

    def mm_per_second_to_arbitrary_unit(self, v, k=0.39, c=0.409):
        """
        Convert speed in mm/s to the arbitrary motor unit using a linear model.
        :param v: Speed in mm/s
        :param k: Slope (conversion factor) in mm/s per arbitrary unit (default: 0.39) (computed from measurement)
        :param c: Offset (constant) in mm/s (default: 0.409) (computed from measurement)
        :return: Speed in arbitrary motor units

        Measurements made :
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

    def set_motor_speed(self, speed_left, speed_right):
        """
        Set the speed of each wheel of the robot.
        :param speed: Integer speed value (in mm/s) [-100:-10] OR [10:100] Negative values move backward for each motor.
        it might work outside theses value but you will see that the conversion is not as precise.
        this function take about 0.1 second
        """

        if self.node is not None:
            v = {
                "motor.left.target": [self.mm_per_second_to_arbitrary_unit(speed_left)],
                "motor.right.target": [self.mm_per_second_to_arbitrary_unit(speed_right)],
            }
            aw(self.node.set_variables(v))
        else:
            print("robot not connected")

    def set_angular_speed(self, angular_speed):
        """
        Turn the robot on the spot by a given angle.
        :param angle: Angle in radians/per second (positive = clockwise rotation).
                between [-4.5,-0.45] AND [0.45, 4.5] radian/s

        the wheel are spaced 95 mm from each other, 47.5mm radius
        for a full turn, each wheel travel along a perimeter of 298.45 mm
        assuming max speed of 100 mm/s for precision, thats 1.5 sec for a full 180° /pi radian

        (was manually adapted to 43 mm since the 47.5mm radius was not producing accurate rotation)
        """
        motor_speed = self.mm_per_second_to_arbitrary_unit(angular_speed*43/(2))
        self.set_motor_speed(motor_speed, -motor_speed)

    def get_temperature(self):
        """
        return the temperature in an arbitrary unit (or in kelvin and really off by 10-20 K)
        """
        if self.node is not None:
            #aw(self.node.lock())
            aw(self.client.wait_for_status(self.client.NODE_STATUS_READY))
            self.client.aw(self.node.wait_for_variables())
            temperature = self.node.v.temperature
            #aw(self.node.unlock())
            return temperature
        else:
            print("robot not connected")

    def get_proximity_ir_sensor(self):
        """
        return a dictionary with the following key :
        the greater the value (0-4096) the closer the obstacle
        """

        if self.node is not None:
            #aw(self.node.lock())
            aw(self.client.wait_for_status(self.client.NODE_STATUS_READY))
            self.client.aw(self.node.wait_for_variables({"prox.horizontal"}))
            sensor = self.node.v.prox.horizontal
            sensorDict = {
                "rear_right":sensor[6],
                "rear_left": sensor[5],
                "front_left": sensor[0],
                "front_center_left": sensor[1],
                "front_center": sensor[2],
                "front_center_right": sensor[3],
                "front_right": sensor[4],
            }
            #aw(self.node.unlock())
            return sensorDict
        else:
            print("robot not connected")

    def get_button_pressed(self):
        """
        return a dictionary with the following key :
        the greater the value (0-4096) the closer the obstacle
        (the button left and right are defined when you look at the robot with the robot pointing aheaf of you.
        """

        if self.node is not None:
            #aw(self.node.lock())
            aw(self.client.wait_for_status(self.client.NODE_STATUS_READY))
            self.client.aw(self.node.wait_for_variables({"button.backward", "button.left", "button.right", "button.forward", "button.center"}))
            buttonDict = {
                "backward":self.node.v.button.backward,
                "left": self.node.v.button.left,
                "center": self.node.v.button.center,
                "forward": self.node.v.button.forward,
                "right": self.node.v.button.right,
            }
            #aw(self.node.unlock())
            return buttonDict
        else:
            print("robot not connected")
    def get_all_variable(self):
        if self.node is not None:
            #aw(self.node.lock())
            aw(self.client.wait_for_status(self.client.NODE_STATUS_READY))
            self.client.aw(self.node.wait_for_variables())
            # Get all attributes of node.v
            variables = dir(self.node.v)

            # Filter out private attributes and methods
            variable_names = list(aw(self.node.var_description()))

            # Retrieve each variable's value
            variable_values = {var: getattr(self.node.v, var) for var in variable_names}

            for i in self.node.v.prox.horizontal:
                print(i)
            #aw(self.node.unlock())
            return variable_values
        else:
            print("robot not connected")


if __name__ == "__main__":
    robot = RobotMovement()
    robot.connect()
    print("setting waypoints")
    robot.set_motor_speed(0.0, 0.0)
    # test case with random waypoints
    robot.set_waypoints([np.array([0.0, 0.0, 0.0]), np.array([0.0, 200.0, 0.0]), np.array([200.0, 400.0, 0.0]), np.array([400.0, 600.0, 0.0]), np.array([600.0, 800.0, 0.0]), np.array([800.0, 1000.0, 0.0])])
    # test case with square waypoints
    # robot.set_waypoints([np.array([200.0, 0.0, 0.0]), np.array([200.0, 200.0, 0.0]), np.array([0.0, 200.0, 0.0]), np.array([0.0, 0.0, 0.0])])
    print("done setting waypoints")
    while True:
        robot.update()
