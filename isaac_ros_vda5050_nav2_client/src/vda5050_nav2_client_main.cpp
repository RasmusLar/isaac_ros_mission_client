// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_vda5050_nav2_client/ackermann_geometry_bridge_node.hpp"
#include "isaac_ros_vda5050_nav2_client/lift_node.hpp"
#include "isaac_ros_vda5050_nav2_client/vda5050_nav2_client_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor exec;
	rclcpp::NodeOptions options;
	auto client_node
		= std::make_shared<isaac_ros::mission_client::Vda5050toNav2ClientNode>(options);
	exec.add_node(client_node);
	auto lift_node = std::make_shared<LiftActionServer>();
	exec.add_node(lift_node);
	auto ackermann_node = std::make_shared<AckermannGeometryBridge>(options);
	exec.add_node(ackermann_node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}
