// SYSTEM
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// ROS headers
#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <urdf/model.h>

// ROS controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_toolbox/filters.h>


namespace fan_hwiface
{

class COPTERHW : public hardware_interface::RobotHW
{
public:

	COPTERHW() {}
	virtual ~COPTERHW() {}

	std::string robot_namespace_;

	std::string urdf_string_;
	urdf::Model urdf_model_;

	realtime_tools::RealtimePublisher<std_msgs::UInt8> *pub_commands_;
	ros::Subscriber sub_measures_;


	bool init(ros::NodeHandle &nh, std::string robot_name)
	{
		n_joints_ = 2;

		pub_commands_ = new realtime_tools::RealtimePublisher<std_msgs::UInt8>(nh, "arduino_cmd", 1);
		sub_measures_ = nh.subscribe("potentiometer_msr", 1000, &COPTERHW::updateMsr, this) ;

		nh.getParam("gain", gain_);
		nh.getParam("bias", bias_);

		robot_namespace_ = robot_name;
		joint_names_.push_back( std::string("free_joint") );
		joint_names_.push_back( std::string("motor_joint") );

		joint_position_.resize(n_joints_);
		joint_position_prev_.resize(n_joints_);
		joint_velocity_.resize(n_joints_);
		joint_velocity_command_.resize(n_joints_);

		joint_lower_limits_.resize(n_joints_);
		joint_upper_limits_.resize(n_joints_);

		for (int j = 0; j < n_joints_; ++j)
		{
			joint_position_[j] = 0.0;
			joint_position_prev_[j] = 0.0;
			joint_velocity_[j] = 0.0;
			joint_velocity_command_[j] = 0.0;
		}

		for(int j=0; j < n_joints_; j++)
		{
			state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_velocity_[j]));

			hardware_interface::JointHandle joint_handle_velocity;
			joint_handle_velocity = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]), &joint_velocity_command_[j]);
			velocity_interface_.registerHandle(joint_handle_velocity);
		}

		registerInterface(&state_interface_);
		registerInterface(&velocity_interface_);

		return true;
	};

	void read(ros::Time time, ros::Duration period)
	{
		// update free joint
		joint_position_prev_[0] = joint_position_[0];
		joint_position_[0] = angle_; // read from potentiometer
		joint_velocity_[0] = filters::exponentialSmoothing((joint_position_[0]-joint_position_prev_[0])/period.toSec(), joint_velocity_[0], 0.2);

		// update motor joint
		joint_position_prev_[1] = joint_position_[1];
		joint_position_[1] += joint_velocity_[1]*period.toSec();
		joint_velocity_[1] = joint_velocity_command_.at(1);
		return;
	};

	void write(ros::Time time, ros::Duration period)
	{
		if (pub_commands_->trylock())
		{
			// only actuate in the correct direction
			double fan_speed = ( ( joint_velocity_command_.at(1) ) );
			if( fan_speed > 0.0 )
				fan_speed = 0;
			fan_speed = std::abs(fan_speed);
			pub_commands_->msg_.data = static_cast<uint8_t>(1023*fan_speed/5);
			if( pub_commands_->msg_.data > 255 )
				pub_commands_->msg_.data = 255;
			pub_commands_->unlockAndPublish();
		}
		return;
	};

	// hardware interfaces
	hardware_interface::JointStateInterface state_interface_;
	hardware_interface::VelocityJointInterface velocity_interface_;


	// joint limits interfaces
	joint_limits_interface::VelocityJointSaturationInterface   vj_sat_interface_;
	joint_limits_interface::VelocityJointSoftLimitsInterface   vj_limits_interface_;
	joint_limits_interface::PositionJointSaturationInterface   pj_sat_interface_;
	joint_limits_interface::PositionJointSoftLimitsInterface   pj_limits_interface_;

	// Before write, you can use this function to enforce limits for all values
	void enforceLimits(ros::Duration period);

	// configuration
	int n_joints_;
	std::vector<std::string> joint_names_;

	// limits
	std::vector<double> 
	joint_lower_limits_,
	joint_upper_limits_;

	// state and commands
	std::vector<double>
	joint_position_,
	joint_position_prev_,
	joint_velocity_,
	joint_velocity_command_;

	// aux var to update the angle
	double angle_;

	// sensor calibration
	double gain_;
	double bias_;

	// set all members to default values
	void reset();

	// transmissions in this plugin's scope
	std::vector<transmission_interface::TransmissionInfo> transmissions_;

private:

	void updateMsr(const std_msgs::UInt16::ConstPtr& msg)
	{
		angle_ = gain_* msg->data + bias_;
	}

/*  // Get Transmissions from the URDF
	bool parseTransmissionsFromURDF(const std::string& urdf_string);

	// Register all interfaces using
	void registerInterfaces(const urdf::Model *const urdf_model, 
									 std::vector<transmission_interface::TransmissionInfo> transmissions);

	// Initialize all KDL members
	bool initKDLdescription(const urdf::Model *const urdf_model);

	// Helper function to register limit interfaces
	void registerJointLimits(const std::string& joint_name,
									 const hardware_interface::JointHandle& joint_handle_effort,
									 const hardware_interface::JointHandle& joint_handle_position,
									 const hardware_interface::JointHandle& joint_handle_velocity,
									 const hardware_interface::JointHandle& joint_handle_stiffness,
									 const urdf::Model *const urdf_model,
									 double *const lower_limit, double *const upper_limit,
									 double *const lower_limit_stiffness, double *const upper_limit_stiffness,
									 double *const effort_limit);*/

}; // class

} // namespace

bool g_quit = false;

void quitRequested(int sig)
{
	g_quit = true;
}

int main( int argc, char** argv )
{
	// initialize ROS
	ros::init(argc, argv, "fan_hwiface", ros::init_options::NoSigintHandler);

	// ros spinner
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// custom signal handlers
	signal(SIGTERM, quitRequested);
	signal(SIGINT, quitRequested);
	signal(SIGHUP, quitRequested);

	// create a node
	ros::NodeHandle copter_arm_nh;

	// get params or give default values
	std::string name;
	copter_arm_nh.param("name", name, std::string("copter_arm"));

	// advertise the e-stop topic
	// ros::Subscriber estop_sub = copter_arm_nh.subscribe(copter_arm_nh.resolveName("emergency_stop"), 1, eStopCB);

	// get the general robot description, the lwr class will take care of parsing what's useful to itself
	// std::string urdf_string = getURDF(copter_arm_nh, "/robot_description");

	// construct and start the real lwr
	fan_hwiface::COPTERHW copter_arm_hwiface;
	// copter_arm_hwiface.create(name, urdf_string);
	if(!copter_arm_hwiface.init(copter_arm_nh, name))
	{
		ROS_FATAL_NAMED("lwr_hw","Could not initialize robot real interface");
		return -1;
	}

	// timer variables
	struct timespec ts = {0, 0};
	ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
	ros::Duration period(1.0);

	//the controller manager
	controller_manager::ControllerManager manager(&copter_arm_hwiface, copter_arm_nh);

	ros::Rate rate(100);
	while( !g_quit )
	{
		// get the time / period
		if (!clock_gettime(CLOCK_REALTIME, &ts))
		{
			now.sec = ts.tv_sec;
			now.nsec = ts.tv_nsec;
			period = now - last;
			last = now;
		} 
		else
		{
			ROS_FATAL("Failed to poll realtime clock!");
			break;
		}

		// read the state from the lwr
		copter_arm_hwiface.read(ros::Time::now(), period);

		// Compute the controller commands
		bool resetControllers = false;
		/*if(!wasStopHandled && !resetControllers)
		{
			ROS_WARN("E-STOP HAS BEEN PRESSED: Controllers will be restarted, but the robot won't move until you release the E-Stop");
			ROS_WARN("HOW TO RELEASE E-STOP: rostopic pub -r 10 /NAMESPACE/emergency_stop std_msgs/Bool 'data: false'");
			resetControllers = true;
			wasStopHandled = true;
		}

		if( isStopPressed )
		{
			wasStopHandled = false;
		}
		else
		{
			resetControllers = false;
			wasStopHandled = true;
		}  */  

		// update the controllers
		manager.update(ros::Time::now(), period, resetControllers);

		// write the command to the lwr
		copter_arm_hwiface.write(ros::Time::now(), period);

		rate.sleep();
	}

	std::cerr<<"Stopping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Bye!"<<std::endl;

	return 0;
}