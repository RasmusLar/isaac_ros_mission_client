

#include "isaac_ros_vda5050_nav2_client/action/lift_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <functional>
#include <memory>
#include <thread>

class LiftActionServer : public rclcpp::Node {
public:
	using Lift			 = isaac_ros_vda5050_nav2_client::action::LiftAction;
	using GoalHandleLift = rclcpp_action::ServerGoalHandle<Lift>;

	explicit LiftActionServer(rclcpp::NodeOptions const& options = rclcpp::NodeOptions())
	: Node("lift_action_server", options)
	, liftCommandPublisher(create_publisher<std_msgs::msg::Float32>("cmd_lift", 1)) {
		using namespace std::placeholders;

		this->action_server_ = rclcpp_action::create_server<Lift>(
			this,
			"lift_action",
			std::bind(&LiftActionServer::handle_goal, this, _1, _2),
			std::bind(&LiftActionServer::handle_cancel, this, _1),
			std::bind(&LiftActionServer::handle_accepted, this, _1));
	}

private:
	rclcpp_action::Server<Lift>::SharedPtr action_server_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr liftCommandPublisher;

	rclcpp_action::GoalResponse handle_goal(
		rclcpp_action::GoalUUID const& /*uuid*/, std::shared_ptr<Lift::Goal const> /*goal*/) {
		RCLCPP_INFO(this->get_logger(), "Received goal request");
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse
	handle_cancel(std::shared_ptr<GoalHandleLift> const /*goal_handle*/) {
		RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted(std::shared_ptr<GoalHandleLift> const goal_handle) {
		using namespace std::placeholders;
		// this needs to return quickly to avoid blocking the executor, so spin up a new thread
		std::thread{std::bind(&LiftActionServer::execute, this, _1), goal_handle}.detach();
	}

	void execute(std::shared_ptr<GoalHandleLift> const goal_handle) {
		RCLCPP_INFO(this->get_logger(), "Executing goal");
		rclcpp::Rate loop_rate(50);
		auto const goal = goal_handle->get_goal();
		float lift_cmd[2];
		auto emptyCallback
			= [&lift_cmd](std_msgs::msg::Float32MultiArray::ConstSharedPtr const msg) {
				  lift_cmd[0] = msg->data[0];
				  lift_cmd[1] = msg->data[1];
			  };
		auto liftCmdSub = create_subscription<std_msgs::msg::Float32MultiArray>(
			"get_lift", rclcpp::SensorDataQoS(), emptyCallback);
		std_msgs::msg::Float32 speed;
		speed.data = 0.0;

		auto clock	  = get_clock();
		auto prevTime = clock->now();

		// parse keys and values

		auto newTime = clock->now();
		auto isClose = [&goal](float const message[2]) {
			return !(
				message[0] < goal->target - goal->target_epsilon
				|| message[0] > goal->target + goal->target_epsilon);
		};
		while (rclcpp::ok() && !isClose(lift_cmd)) {
			auto const distance = goal->target - lift_cmd[0];
			float const deltaT	= (newTime - prevTime).seconds();

			auto const velAcc = ::std::abs(speed.data) + goal->acceleration * deltaT;
			// a = (Vf*Vf - Vi*Vi)/(2 * d), solve for Vi, where Vf is 0.
			auto const velDeacc
				= ::std::sqrt(::std::abs(2 * goal->acceleration * ::std::abs(distance)));

			auto newSpeed = ::std::max(goal->deadband_speed, ::std::min(velAcc, velDeacc));
			if (distance < 0) {
				newSpeed *= -1;
			}

			// RCLCPP_INFO(
			// 	this->get_logger(),
			// 	"New speed %Lf, current loc %f, current speed: %f",
			// 	newSpeed,
			// 	liftCmd[0],
			// 	liftCmd[1]);

			// Limit speed
			speed.data
				= ::std::max<float>(-goal->max_speed, ::std::min<float>(goal->max_speed, newSpeed));
			liftCommandPublisher->publish(speed);

			loop_rate.sleep();

			prevTime = newTime;
			newTime	 = clock->now();
		}
		// Check if goal is done
		auto result		= std::make_shared<Lift::Result>();
		result->success = false;
		liftCmdSub.reset();
		if (!goal_handle->is_canceling()) {
			if (rclcpp::ok()) {
				speed.data = 0.0;
				liftCommandPublisher->publish(speed);
				if (isClose(lift_cmd)) {
					result->success = true;
					RCLCPP_INFO(this->get_logger(), "Goal succeeded");
				} else {
					RCLCPP_INFO(this->get_logger(), "Goal failed");
				}
				goal_handle->succeed(result);
			} else {
				goal_handle->abort(result);
			}
		}
	}
}; // class LiftActionServer

RCLCPP_COMPONENTS_REGISTER_NODE(LiftActionServer)
