#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
 
 
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
 
// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pos = *msg;
}
 
 
float err_x = 0;     float err_x0 = 0;		float err_x_err = 0;
float err_y = 0;	 float err_y0 = 0;		float err_y_err = 0;
float err_z = 0;	 float err_z0 = 0;		float err_z_err = 0;
float sum_ex=0;
float sum_ey=0;
float sum_ez=0;
float vel_x = 0;
float vel_y = 0;
float vel_z = 0;
float kp = 2.0;
float kd = 0.5;  
float ki = 0.01; 
// -0. 2 超调量1 左右   + 0.2 超调量减小
//  0.5-[0.3]   0.8-[??]  1.0-[比0.8 好一些]  
//  设定控制无人机的位置
void local_pos_control(float pos_x, float pos_y, float pos_z)
{
	// 误差
	err_x = pos_x - local_pos.pose.position.x;
	err_y = pos_y - local_pos.pose.position.y;
	err_z = pos_z - local_pos.pose.position.z;
	// 误差的误差
	err_x_err = err_x - err_x0;
	err_y_err = err_y - err_y0;
	err_z_err = err_z - err_z0;
	// 保存本次误差
	err_x0 = err_x;
	err_y0 = err_y;
	err_z0 = err_z;
	sum_ex+=err_x;
	sum_ey+=err_y;
	sum_ez+=err_z;
	vel_x = kp * err_x + kd * err_x_err+ki*sum_ex;
	vel_y = kp * err_y + kd * err_y_err+ki*sum_ey;
	vel_z = kp * err_z + kd * err_y_err+ki*sum_ez;
 
	// 比例控制
	// vel_x = adj_kp * err_x;
	// vel_y = adj_kp * err_y;
	// vel_z = adj_kp * err_z;

	ROS_INFO("Pose-x: %f",local_pos.pose.position.x);
	ROS_INFO("Pose-y: %f",local_pos.pose.position.y);
	ROS_INFO("Pose-z: %f",local_pos.pose.position.z);
}
 
int main(int argc, char **argv)
{   
	ros::init(argc, argv, "offb_cfx");
	ros::NodeHandle nh;
	// 订阅无人机当前状态 
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
			("mavros/state", 10, state_cb);
		// 订阅无人机当前位置（反馈消息） 
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
			("mavros/local_position/pose", 10, local_pos_cb);
	// 发布无人机本地速度（控制）
	ros::Publisher vec_pub = nh.advertise<geometry_msgs::Twist>
			("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	// 服务的客户端（设定无人机的模式、状态）
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
			("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);
 
	// wait for FCU connection
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::Twist vector;
	vector.linear.x = 0.0;
	vector.linear.y = 0.0;
	vector.linear.z = 0.0;

	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		vec_pub.publish(vector);
		ros::spinOnce();
		rate.sleep();
	}
 
	// 设定无人机工作模式 offboard
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	// 无人机解锁
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	// 记录当前时间
	ros::Time last_request = ros::Time::now();

	//  用于走圈的变量
	int step = 0;
	int sametimes = 0;
 
 
    while(ros::ok()){
        // 无人机状态设定与判断      
        // 进入while循环后，先循环5s，然后再向客户端发送无人机状态设置的消息
        // set_mode_client.call   arming_client.call 
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
		{
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
			{
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
		else if(!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if( arming_client.call(arm_cmd) &&
				arm_cmd.response.success)
			{
				ROS_INFO("Vehicle armed");
			}
			last_request = ros::Time::now();
		}
		else if(current_state.mode == "OFFBOARD"&&current_state.armed)
				//  无人机 Offboard enabled && Vehicle armed 后
		{       //  无人机走矩形 每到达一个点停一会
				//  z: 0-->10 10-->10 10-->10 10-->10  10-->10 10-->0 
				//  x: 0-->0   0-->40   40-->40 40-->0    0-->0     0-->0
				//  y: 0-->0   0-->0     0-->20   20-->20  20-->0   0-->0
				//  local_pos_pub.publish(pose);
			
			switch (step)
			{
				case 0:
					local_pos_control(0,0,2);  // 速度控制
					vector.linear.x = vel_x;
					vector.linear.y = vel_y;
					vector.linear.z = vel_z;
					if (local_pos.pose.position.z >1.9 && local_pos.pose.position.z <2.1)
					{
						if (sametimes == 20)
							step = 1;
						else
							sametimes ++;
					}
					else
						sametimes = 0;
					break;

				case 1:
					local_pos_control(10,0,2);
					vector.linear.x = vel_x;
					vector.linear.y = vel_y;
					vector.linear.z = vel_z;
					if (local_pos.pose.position.x >9.9 && local_pos.pose.position.x <10.1)
					{
						if (sametimes == 20)
							step = 2;
						else
							sametimes ++;
					}
					else
						sametimes = 0;
					break;

				case 2:
					local_pos_control(10,10,2);
					vector.linear.x = vel_x;
					vector.linear.y = vel_y;
					vector.linear.z = vel_z;
					if (local_pos.pose.position.y >9.9 && local_pos.pose.position.y <10.1)
					{
						if (sametimes == 20)
							step = 3;
						else
							sametimes ++;
					}
					else
						sametimes = 0;
					break;

				case 3:
					local_pos_control(0,10,2);
					vector.linear.x = vel_x;
					vector.linear.y = vel_y;
					vector.linear.z = vel_z;
					if (local_pos.pose.position.x >-0.1 && local_pos.pose.position.x <0.1)
					{
						if (sametimes == 20)
							step = 4;
						else
							sametimes ++;
					}
					else
						sametimes = 0;
					break;

				case 4:
					local_pos_control(0,0,2);
					vector.linear.x = vel_x;
					vector.linear.y = vel_y;
					vector.linear.z = vel_z;
					if (local_pos.pose.position.y >-0.1 && local_pos.pose.position.y <0.1)
					{
						if (sametimes == 20)
							step = 5;
						else
							sametimes ++;
					}
					else
						sametimes = 0;
					break;

				case 5:    // 准备降落
					offb_set_mode.request.custom_mode = "AUTO.LAND";
					if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
					{
						if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
						{
							ROS_INFO("AUTO.LAND enabled");
							step = 6;
						}
						last_request = ros::Time::now();
					}
					break;

				default:
					break;
			}
			vec_pub.publish(vector); //发布速度控制信息
		}
		else
		{
			vec_pub.publish(vector);
		}
 
        ros::spinOnce();
        rate.sleep();   // 影响消息发布与更新的周期
    }
 
    return 0;
}