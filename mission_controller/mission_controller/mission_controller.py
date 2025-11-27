#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, Twist
from std_msgs.msg import Bool, String

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.state = None
        self.mission = None

        self.declare_parameter('mission', None)
        self.mission = self.get_parameter('mission').value

        self.mission_trigger_track3 = None
        self.mission_trigger_track4 = None

        self.robot_stand_publisher = self.create_publisher(String, '/go2/posture_cmd', 10)
        self.robot_stand_subscriber = self.create_subscription(Bool, '/robot_stand_feedback', self.robot_stand_callback, 10)
        self.robot_stand_state = False

        self.robot_sitdown_publisher = self.create_publisher(String, '/go2/posture_cmd', 10)
        self.robot_sitdown_subscriber = self.create_subscription(Bool, '/robot_sitdown_feedback', self.robot_sitdown_callback, 10)
        self.robot_sitdown_state = False

        self.robot_navigation_publisher = self.create_publisher(String, '/navigation_cmd', 10)
        self.robot_navigation_subscriber = self.create_subscription(Bool, '/robot_navigation_feedback', self.robot_navigation_callback, 10)
        self.robot_navigation_state = False

        self.mission_box_publisher = self.create_publisher(Bool, '/mission_box', 10)
        self.mission_box_subscriber = self.create_subscription(Bool, '/mission_box_feedback', self.mission_box_callback, 10)
        self.mission_box_state = False

        self.mission_button_publisher = self.create_publisher(Bool, '/mission_button', 10)
        self.mission_button_subscriber = self.create_subscription(Bool, '/mission_button_feedback', self.mission_button_callback, 10)
        self.mission_button_state = False

        self.mission_door_publisher = self.create_publisher(Bool, '/mission_door', 10)
        self.mission_door_subscriber = self.create_subscription(Bool, '/mission_door_feedback', self.mission_door_callback, 10)
        self.mission_door_state = False

        self.mission_straight_driving_publisher = self.create_publisher(Bool, '/mission_straight_driving', 10)
        self.mission_straight_driving_subscriber = self.create_subscription(Bool, '/mission_straight_driving_feedback', self.mission_straight_driving_callback, 10)
        self.mission_straight_driving_state = False

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.main_timer = self.create_timer(0.1, self.timer_callback_main)
        
        self.feedback_reset_timer = self.create_timer(1.0, self.reset_feedback_states)
        
        self.get_logger().info('Mission Controller initialized')

    def robot_stand_callback(self, msg):
        if msg.data:
            self.robot_stand_state = True
            self.get_logger().info('Robot standing successfully!')
    
    def robot_sitdown_callback(self, msg):
        if msg.data:
            self.robot_sitdown_state = True
            self.get_logger().info('Robot sitting down successfully!')
    
    def robot_navigation_callback(self, msg):
        self.get_logger().info(f'Robot navigation feedback: {msg.data}')
        if msg.data:
            self.robot_navigation_state = True
            self.get_logger().info('Robot navigating successfully!')

    def mission_box_callback(self, msg):
        if msg.data:
            self.mission_box_state = True
            self.get_logger().info('Box grabbed successfully!')
    
    def mission_button_callback(self, msg):
        if msg.data:
            self.mission_button_state = True
            self.get_logger().info('Mission button pressed successfully!')

    def mission_door_callback(self, msg):
        if msg.data:
            self.mission_door_state = True
            self.get_logger().info('Mission door opened successfully!')
    
    def mission_straight_driving_callback(self, msg):
        if msg.data:
            self.mission_straight_driving_state = True
            self.get_logger().info('Mission straight driving completed successfully!')
    
    def reset_feedback_states(self):
        if self.mission == 'track_4':
            if self.state == 'driving_1':
                pass
            elif self.state == 'robot_sitdown':
                self.robot_navigation_state = False
            elif self.state == 'mission_box':
                self.robot_sitdown_state = False
            elif self.state == 'robot_stand':
                self.mission_box_state = False
            elif self.state == 'driving_2':
                self.robot_stand_state = False

        elif self.mission == 'track_3':
            if self.state == 'mission_button':
                self.robot_sitdown_state = False
            elif self.state == 'robot_stand':
                self.mission_button_state = False
            elif self.state == 'driving_2':
                self.robot_stand_state = False
            elif self.state == 'mission_door':
                self.robot_sitdown_state = False
            elif self.state == 'driving_3':
                self.mission_door_state = False
    
    def timer_callback_main(self):
        # track 1: -----------------------------------------
        if self.mission == 'track_1' and self.state == None:
            self.state = 'driving'

        elif self.mission == 'track_1' and self.state == 'driving':
            self.get_logger().info('Track 1: driving')
            if self.robot_navigation_state:
                self.get_logger().info('Track 1: Completed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                self.mission = None
                self.state = None
            self.robot_navigation_publisher.publish(String(data='path_1'))

        # track 2: -----------------------------------------
        if self.mission == 'track_2' and self.state == None:
            self.state = 'driving'

        elif self.mission == 'track_2' and self.state == 'driving':
            self.get_logger().info('Track 2: driving')
            if self.robot_navigation_state:
                self.mission = None
                self.state = None
            self.robot_navigation_publisher.publish(String(data='path_2'))
        
        # track 3: -----------------------------------------
        if self.mission == 'track_3' and self.state == None:
            self.get_logger().info('Track 3 start')
            self.state = 'driving_1'

        elif self.mission == 'track_3' and self.state == 'driving_1':
            self.get_logger().info('Track 3: driving_1')
            if self.robot_navigation_state:
                self.state = 'robot_sitdown'
                self.mission_trigger_track3 = "first"
                return
            self.robot_navigation_publisher.publish(String(data='path_3_1')) # 시발 이거 파일 이름 바꿔야 함
            # self.robot_navigation_publisher.publish(String(data='path_3_1_modified_1'))
            # self.robot_navigation_publisher.publish(String(data='path_3_1_modified_2'))
            # self.robot_navigation_publisher.publish(String(data='path_3_1_modified_3'))
            # self.robot_navigation_publisher.publish(String(data='path_3_1_modified_4'))

        elif self.mission == 'track_3' and self.state == 'robot_sitdown' and self.mission_trigger_track3 == "first":
            self.robot_navigation_state = False
            self.get_logger().info('Track 3: robot sitdown')
            if self.robot_sitdown_state:
                self.state = 'mission_button'
                self.mission_trigger_track3 = "second"
            self.robot_sitdown_publisher.publish(String(data='stand_down'))

        elif self.mission == 'track_3' and self.state == 'mission_button':
            self.get_logger().info('Track 3: mission button')
            if self.mission_button_state:
                self.state = 'robot_stand_1'
            self.mission_button_publisher.publish(Bool(data=True))

        elif self.mission == 'track_3' and self.state == 'robot_stand_1':
            self.get_logger().info('Track 3: robot stand')
            if self.robot_stand_state:
                self.state = 'driving_2'
            self.robot_stand_publisher.publish(String(data='stand'))

        elif self.mission == 'track_3' and self.state == 'driving_2':
            self.get_logger().info('Track 3: driving_2')
            if self.robot_navigation_state:
                self.state = 'robot_sitdown'
            self.robot_navigation_publisher.publish(String(data='path_3_2'))

        elif self.mission == 'track_3' and self.state == 'robot_sitdown' and self.mission_trigger_track3 == "second":
            self.get_logger().info('Track 3: robot sitdown')
            if self.robot_sitdown_state:
                self.state = 'mission_door'
                self.mission_trigger_track3 = "third"
            self.robot_sitdown_publisher.publish(String(data='stand_down'))

        elif self.mission == 'track_3' and self.state == 'mission_door':
            self.get_logger().info('Track 3: Step 4 _ mission door')
            if self.mission_door_state:
                self.state = 'robot_stand_2'
            self.mission_door_publisher.publish(Bool(data=True))

        elif self.mission == 'track_3' and self.state == 'robot_stand_2':
            self.get_logger().info('Track 3: robot stand')
            if self.robot_stand_state:
                self.state = 'driving_3'
            self.robot_stand_publisher.publish(String(data='stand'))

        elif self.mission == 'track_3' and self.state == 'driving_3':
            self.get_logger().info('Track 3: driving_3')
            if self.robot_navigation_state:
                self.state = 'robot_sitdown'
            self.robot_navigation_publisher.publish(String(data='path_3_3'))

        # track 4: -----------------------------------------
        if self.mission == 'track_4' and self.state == None:
            self.get_logger().info('Track 4: Start track 4 mission')
            self.state = 'driving_1'

        elif self.mission == 'track_4' and self.state == 'driving_1':
            self.get_logger().info('Track 4: driving_1')
            if self.robot_navigation_state:
                self.state = 'robot_sitdown'
                self.mission_trigger_track4 = "first"
                return
            self.robot_navigation_publisher.publish(String(data='path_4_1'))

        elif self.mission == 'track_4' and self.state == 'robot_sitdown' and self.mission_trigger_track4 == "first":
            self.robot_navigation_state = False
            self.get_logger().info('Track 4: robot sitdown')
            if self.robot_sitdown_state:
                self.state = 'mission_box'
                self.mission_trigger_track4 = "second"
            self.robot_sitdown_publisher.publish(String(data='stand_down'))

        elif self.mission == 'track_4' and self.state == 'mission_box':
            self.get_logger().info('Track 4: mission box')
            if self.mission_box_state:
                self.state = 'robot_stand'
            self.mission_box_publisher.publish(Bool(data=True))
                
        elif self.mission == 'track_4' and self.state == 'robot_stand' and self.mission_trigger_track4 == "second":
            self.get_logger().info('Track 4: robot stand')
            if self.robot_stand_state:
                self.state = 'driving_2'
                self.mission_trigger_track4 = "third"
            self.robot_stand_publisher.publish(String(data="stand"))

        elif self.mission == 'track_4' and self.state == 'driving_2':
            self.get_logger().info('Track 4: driving_2')
            if self.robot_navigation_state:
                self.mission = None
                self.state = None
            self.robot_navigation_publisher.publish(String(data='path_4_2'))

        
        

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission Controller is shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
