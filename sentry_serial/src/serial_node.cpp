#include <sentry_serial/serial_node.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sentry_serial/navigation.h>

ros::Publisher navigation_pub;
float smooth_cmd_vel_x;
float smooth_cmd_vel_y;
float cmd_vel_x;
float cmd_vel_y;
float cmd_vel_z;

sentry_serial::navigation navigation;

double roll, pitch, yaw;
bool start=false;
Serial_Package serial_package;

// 接收到订阅的消息后，会进入消息回调函数
void callback(const geometry_msgs::Twist& smooth_cmd_vel)
{

}

void restart_callback(const std_msgs::Bool::ConstPtr& msg) 
{
	if(msg->data)
	{
		serial_package.linear_x=0;
		serial_package.linear_y=0;
		serial_package.angular_z=0;
	}
}

ros::Time last_time;
ros::Time current_time;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
	static double count = 0;
    geometry_msgs::PoseStamped robot_pose_;
    robot_pose_.pose.orientation = msg->pose.pose.orientation;
    // // 获取当前四元数
    // geometry_msgs::Quaternion current_orientation = robot_pose_.pose.orientation;
    
    // // 将当前四元数转换为 tf2::Quaternion 对象
    // tf2::Quaternion quat;
    // tf2::fromMsg(current_orientation, quat);

    // // 将四元数转换为欧拉角 (roll, pitch, yaw)
    // tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if (!start) {
        yaw = 1;
    }
	
	// ROS_WARN("YAW:%f",yaw);
		float vel[]={(float)serial_package.linear_x,
				(float)serial_package.linear_y,
				(float)serial_package.angular_z,
				(float)yaw};

	test_data[0] = {'I'};
	test_data[1] = {'S'};	
	test_data[2] = {'T'};
	
	memcpy(test_data + 3 ,&vel ,sizeof(vel));
	
	test_data[19] = {'A'};  //结束
	test_data[20] = {'A'};  //结束
	test_data[21] = {'A'};  //结束

	// for(uint8_t i = 0;i < 18 ; i++)
	// 	ROS_INFO("data[%d] = %x" ,i , test_data[i]);

    ser.flush ();
    //ser.write(serial_package.Send_Buffer,data_len);
	ser.write(test_data,22);
	navigation.yaw.data=yaw;
	navigation.x.data=serial_package.linear_x;
	navigation.y.data=serial_package.linear_y;
	navigation.z.data=serial_package.angular_z;
	navigation_pub.publish(navigation);
}

void yaw_callback(const std_msgs::Float32& msg)
{
	yaw = msg.data;
}

void mpc_cmd_vel_callback(const geometry_msgs::Twist& cmd_vel)
{
	cmd_vel_x=cmd_vel.linear.x;
	cmd_vel_y=cmd_vel.linear.y;
	cmd_vel_z=cmd_vel.linear.z;

	serial_package.linear_x = cmd_vel_x*0.25;
    serial_package.linear_y = -cmd_vel_y*0.25;
    serial_package.angular_z = cmd_vel_z;
	
    ROS_INFO("serial_package.linear_x:%f",serial_package.linear_x);
	ROS_INFO("serial_package.linear_y:%f",serial_package.linear_y);
}

void pid_cmd_vel_callback(const geometry_msgs::Twist& cmd_vel)
{
	cmd_vel_x=cmd_vel.linear.x;
	cmd_vel_y=cmd_vel.linear.y;
	cmd_vel_z=cmd_vel.angular.z;

	serial_package.linear_x = cmd_vel_x;
    serial_package.linear_y = cmd_vel_y;
    serial_package.angular_z = cmd_vel_z;
	
    // ROS_INFO("\nSend date finished!\n");
}

void start_callback(const std_msgs::Bool& msg)
{
	if(msg.data)
	{
		start = true;
	}
}

int main(int argc, char **argv)  
{  
    ros::init(argc, argv, "serial_node");  
    ros::NodeHandle nh;  
	nh.getParam("cmd_vel_topic",cmd_vel_topic);

	try
	{
		ser.setPort("/dev/ttyACM0");
		//设置要打开串口名称
		ser.setBaudrate(115200);
		//设置波特率
		serial::bytesize_t bytesize = serial::eightbits;
		ser.setBytesize(bytesize);
		//设置数据位
		serial::parity_t parity = serial::parity_none;
		ser.setParity(parity);
		//设置奇偶检验
		serial::stopbits_t stopbits = serial::stopbits_one;
		ser.setStopbits(stopbits) ;
		//设置停止位
		serial::Timeout to = serial::Timeout::simpleTimeout(100);
		ser.setTimeout(to);
		//设置接收字节间隔的超时时间
		ser.open();	
		//打开串口
	}
	catch(serial::IOException &e)
	{
		ROS_ERROR_STREAM("Error to open serial");//串口设备的权限不够
	}
	std::cout<<"33333"<<std::endl;
	if(ser.isOpen())
	{
		ser.flushInput();
		//清空输入的缓存区
		ROS_INFO_STREAM("Sucees to open serial");	
	}
	else
	{
		return -1;
	}
    ROS_INFO_STREAM("Init Finished!");

    // 设置要发送的数据  
    //std::string message = "Hello from ROS!";   
	
    // 创建一个Subscriber，订阅名为smooth_cmd_cel的topic，注册回调函数chatterCallback 
    ros::Subscriber sub = nh.subscribe(cmd_vel_topic, 1000, callback); 
    ros::Subscriber restart_sub = nh.subscribe("MY_ICP/restart", 1000, restart_callback); 
    ros::Subscriber mpc_sub = nh.subscribe("mpc_cmd_vel", 1000, mpc_cmd_vel_callback); 
    ros::Subscriber pid_sub = nh.subscribe("pid_cmd_vel", 1000, pid_cmd_vel_callback); 

    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odom_callback); 
    ros::Subscriber yaw_sub = nh.subscribe("Obstacle_cloudget/yaw_angle", 1000, yaw_callback); 
    ros::Subscriber start_sub = nh.subscribe("MY_ICP/move_base_start", 1000, start_callback); 
    navigation_pub = nh.advertise<sentry_serial::navigation>("navigation",10);
	last_time = ros::Time::now();
    ros::spin();
}