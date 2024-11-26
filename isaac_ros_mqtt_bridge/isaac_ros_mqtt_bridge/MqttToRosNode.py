# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""Implementation for MqttToRosNode."""

import json
import socket
import time

import isaac_ros_mqtt_bridge.MqttBridgeUtils as utils
from isaac_ros_mqtt_bridge.MqttBridgeUtils import convert_dict_keys

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rosbridge_library.internal import message_conversion
from rosbridge_library.internal import ros_loader
from std_msgs.msg import String


class MqttToRosNode(Node):
    """
    Bridge node that converts MQTT messages to ROS messages.

    Bridge node that subscribes to a MQTT channel, translates the received message to ROS,
    and publishes it to a ROS message.
    """

    def __init__(self, name="mqtt_to_ros_bridge_node"):
        """Construct the MqttToRosNode."""
        super().__init__(name)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("interface_name", "uagv"),
                ("major_version", "v2"),
                ("manufacturer", "RobotCompany"),
                ("serial_number", "carter01"),
                ("mqtt_client_name", "MqttBridge"),
                ("mqtt_host_name", "localhost"),
                ("mqtt_port", 1883),
                ("mqtt_transport", "tcp"),
                ("mqtt_ws_path", ""),
                ("mqtt_keep_alive", 60),
                ("ros_publisher_type", "vda5050_msgs/Action"),
                ("ros_publisher_queue", 10),
                ("convert_camel_to_snake", True),
                ("reconnect_period", 5),
                ("retry_forever", False),
                ("num_retries", 10),
            ],
        )

        self.mqtt_client = mqtt.Client(
            self.get_parameter("mqtt_client_name").value,
            transport=self.get_parameter("mqtt_transport").value,
        )

        if (
            self.get_parameter("mqtt_transport").value == "websockets"
            and self.get_parameter("mqtt_ws_path").value != ""
        ):
            self.mqtt_client.ws_set_options(
                path=self.get_parameter("mqtt_ws_path").value
            )

        self.interface_name = self.get_parameter("interface_name").value
        self.major_version = self.get_parameter("major_version").value
        self.manufacturer = self.get_parameter("manufacturer").value
        self.serial_number = self.get_parameter("serial_number").value
        self.mqtt_topic_prefix = f"{self.interface_name}/{self.major_version}/{self.manufacturer}/{self.serial_number}"

        def on_mqtt_connect(client, userdata, flags, rc):
            self.get_logger().info(f"Connected with result code {str(rc)}")
            self.mqtt_client.subscribe(f"{self.mqtt_topic_prefix}/order")
            self.mqtt_client.subscribe(f"{self.mqtt_topic_prefix}/instantActions")
            connection_message = utils.ConnectionMessage(
                self.manufacturer, self.serial_number, utils.State.ONLINE
            )

            self.mqtt_client.publish(
                f"{self.mqtt_topic_prefix}/connection",
                str(connection_message),
                qos=1,
                retain=True,
            )

        def on_mqtt_disconnect(client, userdata, rc):
            if rc != 0:
                self.get_logger().info(f"Disconnected with result code {str(rc)}")

        self.mqtt_client.on_connect = on_mqtt_connect
        self.mqtt_client.on_disconnect = on_mqtt_disconnect

        def on_mqtt_message(client, userdata, msg):
            try:
                publisher = None
                self.get_logger().info(f"From {msg.topic}: {str(msg.payload)}")
                if msg.topic.endswith("order"):
                    ros_msg = ros_loader.get_message_instance(
                        self.get_parameter("ros_publisher_type").value
                    )
                    publisher = self.publisher
                if msg.topic.endswith("instantActions"):
                    ros_msg = ros_loader.get_message_instance(
                        "vda5050_msgs/InstantActions"
                    )
                    publisher = self.instant_actions_publisher

                if self.get_parameter("convert_camel_to_snake").value:
                    message_dict = json.loads(str(msg.payload, "utf-8"))
                    converted_message_dict = convert_dict_keys(
                        message_dict, "camel_to_snake"
                    )
                    message_conversion.populate_instance(
                        converted_message_dict, ros_msg
                    )
                else:
                    message_conversion.populate_instance(
                        json.loads(str(msg.payload, "utf-8")), ros_msg
                    )
                if publisher:
                    publisher.publish(ros_msg)
            except (
                message_conversion.FieldTypeMismatchException,
                json.decoder.JSONDecodeError,
            ) as e:
                self.get_logger().info(repr(e))
                error_msg = String()
                error_msg.data = repr(e)
                self.error_publisher.publish(error_msg)

        self.mqtt_client.on_message = on_mqtt_message

        self.publisher = self.create_publisher(
            ros_loader.get_message_class(
                self.get_parameter("ros_publisher_type").value
            ),
            "bridge_pub_topic",
            self.get_parameter("ros_publisher_queue").value,
        )

        self.instant_actions_publisher = self.create_publisher(
            ros_loader.get_message_class("vda5050_msgs/InstantActions"),
            "instant_actions_commands",
            self.get_parameter("ros_publisher_queue").value,
        )

        self.error_publisher = self.create_publisher(
            String, "order_valid_error", self.get_parameter("ros_publisher_queue").value
        )
        # Set a Will message to be sent by the broker in case of disconnection unexpectedly.
        will_message = utils.ConnectionMessage(
            self.manufacturer, self.serial_number, utils.State.CONNECTIONBROKEN
        )
        msg_payload = str(will_message)
        self.mqtt_client.will_set(
            f"{self.mqtt_topic_prefix}/connection",
            payload=msg_payload,
            qos=1,
            retain=True,
        )

        max_retries = self.get_parameter("num_retries").value
        retries = 0
        connected = False
        retry_forever = self.get_parameter("retry_forever").value
        while retries < max_retries or retry_forever:
            try:
                self.mqtt_client.connect(
                    self.get_parameter("mqtt_host_name").value,
                    self.get_parameter("mqtt_port").value,
                    self.get_parameter("mqtt_keep_alive").value,
                )
                connected = True
                break
            except ConnectionRefusedError as e:
                self.get_logger().error(
                    f"Connection Error: {e}. Please check the mqtt_host_name."
                )
                time.sleep(self.get_parameter("reconnect_period").value)
                retries += 1
            except socket.timeout as e:
                self.get_logger().error(
                    f"Connection Error: {e}. Please check the mqtt_host_name"
                    " and make sure it is reachable."
                )
                time.sleep(self.get_parameter("reconnect_period").value)
                retries += 1
            except socket.gaierror as e:
                self.get_logger().error(
                    f"Connection Error: {e}. Could not resolve mqtt_host_name"
                )
                time.sleep(self.get_parameter("reconnect_period").value)
                retries += 1
        if connected:
            self.mqtt_client.loop_start()
        else:
            self.get_logger().error("Failed to connect to MQTT broker, ending retries.")

    def disconnect(self):
        """Disconnect mqtt client and send OFFLINE message."""
        disconnect_message = utils.ConnectionMessage(
            self.manufacturer, self.serial_number, utils.State.OFFLINE
        )
        self.mqtt_client.publish(
            f"{self.mqtt_topic_prefix}/connection", str(disconnect_message), 1, True
        )
        self.mqtt_client.disconnect()
        self.mqtt_client.loop_stop()


def main(args=None):
    """Execute the MqttToRosNode."""
    rclpy.init(args=args)
    node = MqttToRosNode("mqtt_to_ros_bridge_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.disconnect()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
