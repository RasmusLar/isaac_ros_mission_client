#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <limits>

using std::placeholders::_1;

constexpr char const* parameters[5]
	= {"wheel_base",
	   "track_width",
	   "turning_wheel_radius",
	   "max_wheel_velocity",
	   "max_wheel_rotation_angle"};

class AckermannGeometryBridge : public rclcpp::Node {
	using JointState = sensor_msgs::msg::JointState;
	using Twist		 = geometry_msgs::msg::Twist;

public:
	explicit AckermannGeometryBridge(rclcpp::NodeOptions const& options = rclcpp::NodeOptions())
	: Node("ackermann_geometry_bridge", options)
	, jointStatesPublisher(create_publisher<JointState>("cmd_joints", 5))
	, cmdVelSubscriber(create_subscription<Twist>(
		  "cmd_vel", 5, std::bind(&AckermannGeometryBridge::cmdVelCallback, this, _1)))
	, jointStates()
	, wheelBase(::std::nan("0")) {
		RCLCPP_DEBUG(this->get_logger(), "Starting Ackermann!");
		for (auto&& param : parameters) {
			this->declare_parameter(param, rclcpp::PARAMETER_DOUBLE);
		}
		this->declare_parameter("front_left_name", rclcpp::PARAMETER_STRING);
		this->declare_parameter("front_right_name", rclcpp::PARAMETER_STRING);

		jointStates.name	 = {"frontLeft", "frontRight"};
		jointStates.position = {0, 0};
		jointStates.velocity = {0, 0};
	}

	~AckermannGeometryBridge() { RCLCPP_DEBUG(this->get_logger(), "Stopping Ackermann!"); }

private:
	rclcpp::Publisher<JointState>::SharedPtr jointStatesPublisher;

	rclcpp::Subscription<Twist>::SharedPtr cmdVelSubscriber;
	JointState jointStates;
	double wheelBase;
	double trackWidth;
	double turningWheelRadius;
	double maxWheelVelocity;
	double maxWheelRotationAngle;

	void compute_wheel_angles(double& steeringAngle) {
		auto radius = (wheelBase) / ::std::tan(steeringAngle);

		// Equations were simplied from the ones shown in
		// 	https: // www.mathworks.com/help/vdynblks/ref/kinematicsteering.html
		// compute the wheel angles by taking into account their offset from the center of the
		// turning axle(where the bicycle model is centered), then computing the angles of each
		// wheel relative to the turning point of the robot.

		auto& leftWheelAngle  = jointStates.position[0];
		auto& rightWheelAngle = jointStates.position[1];

		leftWheelAngle	= ::std::atan(wheelBase / (radius - 0.5 * trackWidth));
		rightWheelAngle = ::std::atan(wheelBase / (radius + 0.5 * trackWidth));
		// Aligning direction for front wheel drive from limited range of arctan
		// Add/Substract pi if wheel angle is opposite of steering angle
		if (steeringAngle * rightWheelAngle < 0) {
			rightWheelAngle += (rightWheelAngle < 0) ? M_PI : -M_PI;
		}
		if (steeringAngle * leftWheelAngle < 0) {
			leftWheelAngle += (leftWheelAngle < 0) ? M_PI : -M_PI;
		}

		auto const currentMaxWheelAngle
			= ::std::max(::std::abs(leftWheelAngle), ::std::abs(rightWheelAngle));
		if (maxWheelRotationAngle < currentMaxWheelAngle) {
			// clamp wheel angles to max wheel rotation, calculate new steering angle with inverting
			// above calcs:
			auto const scalar = maxWheelRotationAngle / currentMaxWheelAngle;
			auto trackDist	  = 0.5 * trackWidth;
			auto angle		  = rightWheelAngle * scalar;
			if (::std::abs(leftWheelAngle) == currentMaxWheelAngle) {
				angle = leftWheelAngle * scalar;
				trackDist *= -1;
			}
			// angle = arctan(wheel_base / (r + track_dist))
			// // arctan both
			// tan(angle) = wheel_base / (r + track_dist)
			// // move R over to left
			// r + track_dist = wheel_base / tan(angle)
			// r = wheel_base / tan(angle) -  track_dist
			// // break r into command calc
			// inv * wheel_base / tan(command) = wheel_base / tan(angle) - track_dist
			// // div by wheel_base
			// inv / tan(command) = (1 / tan(angle)) - (track_dist / wheel_base)
			// // move tan(command) separate
			// tan(command) = inv / ((1 / tan(angle)) - (track_dist / wheel_base))
			// command = arctan(inv / ((1 / tan(angle)) - (track_dist / wheel_base))) arc tan both
			// and
			// (/ -1 or 1) == (* -1 or 1)
			auto newAngle = ::std::atan(1 / ((1 / ::std::tan(angle)) - (trackDist / wheelBase)));
			// Realign direction from arctan
			if ((steeringAngle * newAngle) < 0) {
				newAngle += (newAngle < 0) ? M_PI : -M_PI;
			}
			RCLCPP_DEBUG(this->get_logger(), "Recalc %f!", steeringAngle);
			steeringAngle = newAngle;
			compute_wheel_angles(steeringAngle);
		} else {
			RCLCPP_DEBUG(this->get_logger(), "Returning %f!", steeringAngle);
		}
	}

	bool validParameters() {
		return ::std::isfinite(wheelBase) && ::std::isfinite(trackWidth)
			   && ::std::isfinite(turningWheelRadius) && wheelBase >= 0 && trackWidth >= 0
			   && turningWheelRadius > 0 && maxWheelVelocity > 0 && maxWheelRotationAngle > 0;
	}

	void cmdVelCallback(Twist const& cmdVel) {
		if (!validParameters()) {
			uint i			   = 0;
			wheelBase		   = this->get_parameter_or(parameters[i++], ::std::nan("0"));
			trackWidth		   = this->get_parameter_or(parameters[i++], ::std::nan("0"));
			turningWheelRadius = this->get_parameter_or(parameters[i++], ::std::nan("0"));
			maxWheelVelocity   = this->get_parameter_or(
				  parameters[i++], ::std::numeric_limits<double>::infinity());
			maxWheelRotationAngle = this->get_parameter_or(parameters[i++], M_PI);
			jointStates.name[0]	  = this->get_parameter_or("front_left_name", jointStates.name[0]);
			jointStates.name[1]	  = this->get_parameter_or("front_right_name", jointStates.name[1]);

			if (!validParameters()) {
				RCLCPP_DEBUG(
					this->get_logger(),
					"Missing or invalid params:\n"
					"Wheel base: %f \t Track width: %f\n"
					"Turning wheel radius: %f \t Max wheel vel: %f\n"
					"Max wheel angle: %f.\n"
					"Unsubscribing from cmd_vel.",
					wheelBase,
					trackWidth,
					turningWheelRadius,
					maxWheelVelocity,
					maxWheelRotationAngle);
				cmdVelSubscriber.reset();
				jointStatesPublisher.reset();
				return;
			}

			RCLCPP_DEBUG(
				this->get_logger(),
				"Params: %f, %f, %f, %f, %f,",
				wheelBase,
				trackWidth,
				turningWheelRadius,
				maxWheelVelocity,
				maxWheelRotationAngle);
		}

		auto linearVel		   = cmdVel.linear.x;
		auto const& angularVel = cmdVel.angular.z;
		auto centreTurningVel  = wheelBase * angularVel;

		auto steeringAngle = ::std::atan2(centreTurningVel, linearVel);
		if (linearVel < 0) {
			steeringAngle += steeringAngle > 0 ? -M_PI : M_PI;
		}
		// Steering angle is modified if it's clipped due to max angles.
		compute_wheel_angles(steeringAngle);

		auto const wheelDistance
			= ::std::sqrt(wheelBase * wheelBase + (trackWidth * trackWidth / 4));
		auto const wheelAngle = ::std::atan((trackWidth / 2) / wheelBase);
		jointStates.velocity[0]
			= ::std::sqrt(
				  ::std::pow(linearVel + ::std::sin(-wheelAngle) * wheelDistance * angularVel, 2.0)
				  + ::std::pow(::std::cos(-wheelAngle) * wheelDistance * angularVel, 2))
			  / turningWheelRadius;
		jointStates.velocity[1]
			= ::std::sqrt(
				  ::std::pow(linearVel + ::std::sin(wheelAngle) * wheelDistance * angularVel, 2.0)
				  + ::std::pow(::std::cos(wheelAngle) * wheelDistance * angularVel, 2))
			  / turningWheelRadius;

		if (jointStates.velocity[0] > maxWheelVelocity) {
			jointStates.velocity[1] *= maxWheelVelocity / jointStates.velocity[0];
			jointStates.velocity[0] = maxWheelVelocity;
		}
		if (jointStates.velocity[1] > maxWheelVelocity) {
			jointStates.velocity[0] *= maxWheelVelocity / jointStates.velocity[1];
			jointStates.velocity[1] = maxWheelVelocity;
		}

		if (linearVel < 0) {
			jointStates.velocity[0] *= -1;
			jointStates.velocity[1] *= -1;
		}

		jointStatesPublisher->publish(jointStates);
	}
}; // class AckermannGeometryBridge

RCLCPP_COMPONENTS_REGISTER_NODE(AckermannGeometryBridge)
