#!/usr/bin/env python3

import math
import rospy
import sys
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class GetVoiceCommands:
    """
    A class that subscribes to the speech to text recognition messages, prints
    a voice command menu, and defines step size for translational and rotational
    mobile base motion.
    """
    def __init__(self):
        """
        A function that initializes subscribers and defines the three different
        step sizes.
        :param self: The self reference.
        """
        self.step_size = 'medium'
        self.rad_per_deg = math.pi/180.0

        self.small_deg = 5.0
        self.small_rad = self.rad_per_deg * self.small_deg
        self.small_translate = 0.025

        self.medium_deg = 10.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.05

        self.big_deg = 20.0
        self.big_rad = self.rad_per_deg * self.big_deg
        self.big_translate = 0.1

        self.voice_command = None
        self.command_list = None
        self.sound_direction = 0
        self.speech_to_text_sub  = rospy.Subscriber("/speech_to_text",  SpeechRecognitionCandidates, self.callback_speech)
        self.sound_direction_sub = rospy.Subscriber("/sound_direction", Int32,                       self.callback_direction)

    def callback_direction(self, msg):
        """
        A callback function that converts the incoming message, sound direction,
        from degrees to radians.
        :param self: The self reference.
        :param msg: The Int32 message type that represents the sound direction.
        """
        self.sound_direction = msg.data * -self.rad_per_deg

    def callback_speech(self,msg):
        """
        A callback function takes the incoming message, a list of the speech to
        text, and joins all items in that iterable list into a single string.
        :param self: The self reference.
        :param msg: The SpeechRecognitionCandidates message type.
        """
        self.voice_command = ' '.join(map(str,msg.transcript))
        print(self.voice_command)
        self.command_list = self.voice_command.split(" ")


    def get_inc(self):
        """
        A function that sets the increment size for translational and rotational
        base motion.
        :param self:The self reference.

        :returns inc: A dictionary type the contains the increment size.
        """
        translation = self.medium_translate
        rotation = self.medium_rad
        if self.command_list:
            for s in self.command_list:
                if s.isnumeric():
                    translation = int(s)
        
        translation = translation /100
        if 'meter' in self.command_list or 'm' in self.command_list or 'M' in self.command_list:
            translation = translation*100


        if 'degrees' in self.command_list:
            rotation = translation*math.pi/180


        inc = {'rad': rotation, 'translate': translation}
        
        return inc

    def print_commands(self):
        """
        A function that prints the voice teleoperation menu.
        :param self: The self reference.
        """
        print('                                           ')
        print('------------ VOICE TELEOP MENU ------------')
        print('                                           ')
        print('               VOICE COMMANDS              ')
        print(' "forward": BASE FORWARD                   ')
        print(' "back"   : BASE BACK                      ')
        print(' "left"   : BASE ROTATE LEFT               ')
        print(' "right"  : BASE ROTATE RIGHT              ')
        print(' "stretch": BASE ROTATES TOWARDS SOUND     ')
        print('                                           ')
        print('                 STEP SIZE                 ')
        print(' "big"    : BIG                            ')
        print(' "medium" : MEDIUM                         ')
        print(' "small"  : SMALL                          ')
        print('                                           ')
        print('                                           ')
        print(' "quit"   : QUIT AND CLOSE NODE            ')
        print('                                           ')
        print('-------------------------------------------')
        print(self.voice_command)
        print(self.command_list)
        if self.command_list:
            if 'meter' in self.command_list or 'm' in self.command_list or 'M' in self.command_list:
                print('I heard you say the word meter')

    def get_command(self):
        """
        A function that defines the teleoperation command based on the voice command.
        :param self: The self reference.

        :returns command: A dictionary type that contains the type of base motion.
        """
        # if forward

        command = None
        if self.voice_command and self.command_list:
            if 'forward' in self.command_list:
                command = {'joint': 'translate_mobile_base', 'inc': self.get_inc()['translate']}
            if 'back' in self.command_list:
                command = {'joint': 'translate_mobile_base', 'inc': -self.get_inc()['translate']}
            if 'left' in self.command_list:
                command = {'joint': 'rotate_mobile_base', 'inc': self.get_inc()['rad']}
            if 'right' in self.command_list:
                command = {'joint': 'rotate_mobile_base', 'inc': -self.get_inc()['rad']}

        '''if (self.voice_command == "small") or (self.voice_command == "medium") or (self.voice_command == "big"):
            self.step_size = self.voice_command
            rospy.loginfo('Step size = {0}'.format(self.step_size))'''

        if self.voice_command == 'quit':
            rospy.signal_shutdown("done")
            sys.exit(0)

        self.voice_command = None
        self.command_list = None
        return command


class VoiceTeleopNode(hm.HelloNode):
    """
    A class that inherits the HelloNode class from hm and sends joint trajectory
    commands.
    """
    def __init__(self):
        """
        A function that declares object from the GetVoiceCommands class, instantiates
        the HelloNode class, and set the publishing rate.
        :param self: The self reference.
        """
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_state = None
        self.speech = GetVoiceCommands()

    def joint_states_callback(self, msg):
        """
        A callback function that stores Stretch's joint states.
        :param self: The self reference.
        :param msg: The JointState message type.
        """
        self.joint_state = msg

    def send_command(self, command):
        """
        Function that makes an action call and sends base joint trajectory goals
        :param self: The self reference.
        :param command: A dictionary type.
        """
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]

            inc = command['inc']
            rospy.loginfo('inc = {0}'.format(inc))
            new_value = inc

            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            rospy.loginfo('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
            self.trajectory_client.send_goal(trajectory_goal)
            rospy.loginfo('Done sending command.')
            self.speech.print_commands()

    def main(self):
        """
        The main function that instantiates the HelloNode class, initializes the subscriber,
        and call other methods in both the VoiceTeleopNode and GetVoiceCommands classes.
        :param self: The self reference.
        """
        hm.HelloNode.main(self, 'voice_teleop', 'voice_teleop', wait_for_first_pointcloud=False)
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        rate = rospy.Rate(self.rate)
        self.speech.print_commands()

        while not rospy.is_shutdown():
            command = self.speech.get_command()
            self.send_command(command)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = VoiceTeleopNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
