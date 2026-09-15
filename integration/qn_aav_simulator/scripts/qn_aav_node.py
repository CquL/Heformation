#!/usr/bin/env python3
"""Execute Swarm-Formation PositionCommand through the original qn closed loop."""

from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
import rospy
from std_msgs.msg import Float64

from qn_aav_simulator.contracts import (
    AgentState,
    CommandMode,
    ControlCmd,
    PlantStepInput,
    PlatformAdapterCmd,
)
from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend


class QnAavNode:
    def __init__(self):
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        self.agent_id = "drone_{}".format(self.drone_id)
        self.dt = 1.0 / float(rospy.get_param("~rate", 100.0))
        position = (
            float(rospy.get_param("~init_x", 0.0)),
            float(rospy.get_param("~init_y", 0.0)),
            float(rospy.get_param("~init_z", 0.5)),
        )
        self.reference_position = position
        self.reference_velocity = (0.0, 0.0, 0.0)
        self.reference_acceleration = (0.0, 0.0, 0.0)
        self.reference_yaw = 0.0
        self.step_index = 0
        self.state = AgentState(
            agent_id=self.agent_id,
            type="AAV",
            timestamp_s=rospy.Time.now().to_sec(),
            position=position,
            velocity=(0.0, 0.0, 0.0),
        )
        self.backend = QnPythonClosedLoopBackend(
            {
                "initialization_mode": "STATIC_TRIM",
                "model_step_s": 0.001,
                "reference_mode": "ROUTE_POSITION",
                "water_guidance_mode": "QN_ORIGINAL_POSITION",
                "water_horizontal_controller_mode": "QN_ORIGINAL_RBF_PD",
            }
        )
        self.backend.reset(self.state)
        self.odom_pub = rospy.Publisher("~odometry", Odometry, queue_size=1)
        self.medium_pub = rospy.Publisher("~medium_flag", Float64, queue_size=1)
        self.command_sub = rospy.Subscriber(
            "~command", PositionCommand, self.command_callback, queue_size=1
        )
        rospy.loginfo(
            "QN AAV %s ready: original RBF/PD + actuators + 6DOF", self.agent_id
        )

    def command_callback(self, message):
        self.reference_position = (
            message.position.x,
            message.position.y,
            message.position.z,
        )
        self.reference_velocity = (
            message.velocity.x,
            message.velocity.y,
            message.velocity.z,
        )
        self.reference_acceleration = (
            message.acceleration.x,
            message.acceleration.y,
            message.acceleration.z,
        )
        self.reference_yaw = message.yaw
    def step(self):
        now = rospy.Time.now()
        command = ControlCmd(
            cmd_id="swarm-formation:{}:{}".format(self.agent_id, self.step_index),
            agent_id=self.agent_id,
            timestamp_s=now.to_sec(),
            command_mode=CommandMode.DESIRED_POSITION,
            desired_position=self.reference_position,
            desired_velocity=self.reference_velocity,
            desired_acceleration=self.reference_acceleration,
            desired_yaw_rad=self.reference_yaw,
        )
        result = self.backend.step(
            PlantStepInput(
                state=self.state,
                control_cmd=command,
                platform_adapter=PlatformAdapterCmd(self.agent_id),
                dt_s=self.dt,
                max_speed_mps=2.0,
                max_acc_mps2=8.0,
            )
        )
        self.step_index += 1
        self.state = AgentState(
            agent_id=self.agent_id,
            type="AAV",
            timestamp_s=now.to_sec(),
            position=result.position,
            velocity=result.velocity,
            acceleration=result.acceleration,
            orientation_quat_wxyz=result.orientation_quat_wxyz,
            body_linear_velocity_mps=result.body_linear_velocity_mps,
            body_angular_velocity_radps=result.body_angular_velocity_radps,
            medium_flag=result.medium_flag,
        )
        self.publish(now)

    def publish(self, stamp):
        result = self.state
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = self.agent_id + "/base_link"
        odom.pose.pose.position.x = result.position[0]
        odom.pose.pose.position.y = result.position[1]
        odom.pose.pose.position.z = result.position[2]
        w, x, y, z = result.orientation_quat_wxyz
        odom.pose.pose.orientation.w = w
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.twist.twist.linear.x = result.velocity[0]
        odom.twist.twist.linear.y = result.velocity[1]
        odom.twist.twist.linear.z = result.velocity[2]
        odom.twist.twist.angular.x = result.body_angular_velocity_radps[0]
        odom.twist.twist.angular.y = result.body_angular_velocity_radps[1]
        odom.twist.twist.angular.z = result.body_angular_velocity_radps[2]
        self.odom_pub.publish(odom)
        self.medium_pub.publish(Float64(data=float(result.medium_flag or 0.0)))

    def run(self):
        rate = rospy.Rate(round(1.0 / self.dt))
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


def main():
    rospy.init_node("qn_aav_simulator")
    QnAavNode().run()


if __name__ == "__main__":
    main()
