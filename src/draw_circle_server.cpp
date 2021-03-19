#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

// srv폴더에 만든 서비스 파일에서 생성되는 헤더파일로 catkin_make 실행 중 생성된다.
#include "turtlesim_autodraw/circle.h"

// 기본 속도; 너무 빠르면 전진 거리/회전 각도가 부정확해진다.
const double speed = 1;
double angular_speed = 1;

// main함수만이 아닌 Service Callback 함수에서도 publisher를 참조해야 하므로 전역 변수로 설정
// NodeHandler를 전역변수로 설정할 경우 ros::init이 실행되기 전 초기화를 시도하면서 오류가 발생한다.
ros::Publisher pub;


// 직선 그리기 (side_length = 변 길이; &vel_msg = 속도 메시지(토픽); &pub = vel_msg 퍼블리셔)
void draw_a_circle(float radius, int rotations, geometry_msgs::Twist &vel_msg, ros::Publisher &pub)
{
	angular_speed = speed / radius; // 원을 그리는 핵심 설정 부분

	// vel_msg에 포함된 3차원 속도 값 초기화
	vel_msg.linear.x = speed;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = angular_speed;

	// 출발 시각
	double t0 = ros::Time::now().toSec();
	// 회전한 각도
	double rotated_angle = 0;

	while (rotated_angle < 2 * M_PI * rotations) // 회전한 각도가 입력한 회전수에 아직 미치지 못했을 경우
	{
		// 속도 메시지 전송(전진 및 회전 명령)
		pub.publish(vel_msg);
		// 짧은 이동 시간 측정하여 회전 각도(= 각속도 * 시간) 계산
		double t1 = ros::Time::now().toSec();
		rotated_angle = angular_speed * (t1 - t0);
	}
	// 전진 속도 및 각속도 모두 0으로 변경; 정지 명령
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;

	pub.publish(vel_msg);
}

// 원 그리기 서비스 Request에 응답하는 Callback 함수
bool handlecircle(turtlesim_autodraw::circle::Request &req, turtlesim_autodraw::circle::Response &res)
{
	ROS_INFO("[draw_circle_server] I just got incoming request!");
	
	// vel_msg 메시지 객체 생성
	geometry_msgs::Twist vel_msg;

	// service client로부터 요청된 반지름 길이 받아오기
	float radius = req.radius;
	ROS_INFO_STREAM("[draw_circle_server] req.radius = " << req.radius);

	// service client로부터 요청된 반복 횟수 받아오기
	int rotations = req.rotations;
	ROS_INFO_STREAM("[draw_circle_server] req.rotations = " << req.rotations);
	
	if (ros::ok())
	{
		// 원 그리기
		draw_a_circle(radius, rotations, vel_msg, pub);

		// 디버그 정보 출력
		// ROS_DEBUG_STREAM("");
	}

	// 여기서 return값에 의미는 없으나 콜백함수 형식을 바꿀 수 없으므로 return 값을 임의로 true로 설정
	return true;
}


int main(int argc, char **argv)
{
	// 노드/노드핸들 초기화
	ros::init(argc, argv, "draw_circle_server");
	ros::NodeHandle nh;
	ROS_INFO("[draw_circle_server] Startup");

	// 노드 반복 실행(spin) 주기 설정(ms)
	ros::Rate loop_rate(10);

	// 퍼블리셔 실행 (turtlesim 조종용 토픽 퍼블리시)
	pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ROS_INFO("[draw_circle_server] Advertising publisher /turtle1/cmd_vel");

	// 서비스 서버 실행 (그릴 사각형 크기와 반복회수 받는 서비스 서버)
	ros::ServiceServer s = nh.advertiseService<turtlesim_autodraw::circle::Request, turtlesim_autodraw::circle::Response>("draw_circle", handlecircle);
	ROS_INFO("[draw_circle_server] Advertising service server draw_circle");

	// CTRL+C 등 종료 명령 받기 위해 ros::spin()대신 ros::ok(), 루프, ros::spinOnce() 조합
	while(ros::ok()) ros::spinOnce();

	return 0;
}