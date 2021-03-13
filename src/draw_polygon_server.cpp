#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

// srv폴더에 만든 서비스 파일에서 생성되는 헤더파일로 catkin_make 실행 중 생성된다.
#include "turtlesim_autodraw/polygon.h"

// 기본 속도; 너무 빠르면 전진 거리/회전 각도가 부정확해진다.
const double speed = 1;
const double angular_speed = 1;

// main함수만이 아닌 Service Callback 함수에서도 publisher를 참조해야 하므로 전역 변수로 설정
// NodeHandler를 전역변수로 설정할 경우 ros::init이 실행되기 전 초기화를 시도하면서 오류가 발생한다.
ros::Publisher pub;


// 직선 그리기 (side_length = 변 길이; &vel_msg = 속도 메시지(토픽); &pub = vel_msg 퍼블리셔)
void draw_in_line(float side_length, geometry_msgs::Twist &vel_msg, ros::Publisher &pub)
{
	// vel_msg에 포함된 3차원 속도 값 초기화
	vel_msg.linear.x = speed; //turtle은 전진만 하므로 x를 제외한 나머지 속도를 0으로 맞춘다.
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	// 출발 시각
	double t0 = ros::Time::now().toSec();
	// 이동 거리
	double distance_travelled = 0;

	while (distance_travelled < side_length) // 이동 거리가 아직 변 길이에 미치지 못했을 경우
	{
		// 속도 메시지 전송(전진 명령)
		pub.publish(vel_msg);
		// 짧은 이동 시간 측정하여 이동 거리(= 속도 * 시간) 계산
		double t1 = ros::Time::now().toSec();
		distance_travelled = speed * (t1 - t0);
	}
	// 전진 속도 0으로 변경; 정지 명령
	vel_msg.linear.x = 0;
	pub.publish(vel_msg);
}

// 회전하기 (side_num = 변 개수; &vel_msg = 속도 메시지(토픽); &pub = vel_msg 퍼블리셔)
void rotate(int side_num, geometry_msgs::Twist &vel_msg, ros::Publisher &pub)
{
	// vel_msg에 포함된 3차원 속도 값 중 각속도만 초기화 (나머지 속도값은 draw_in_line에서 초기화됨)
	vel_msg.angular.z = angular_speed;

	// 출발 시각
	double t0 = ros::Time::now().toSec();

	// 회전한 각도
	double angle_travelled = 0;

	// 회전할 각도: 2π / 변 개수
	const double turn_angle = 2 * M_PI / side_num;

	while (angle_travelled < turn_angle) // 회전한 각도가 목표 각도에 미치지 못했을 경우
	{
		// 속도 메시지 전송(전진 명령)
		pub.publish(vel_msg);
		// 짧은 회전 시간 측정하여 회전 각도(= 각속도 * 시간) 계산
		double t1 = ros::Time::now().toSec();
		angle_travelled = angular_speed * (t1 - t0);
	}
	// 각속도 0으로 변경; 정지 명령
	vel_msg.angular.z = 0;
	pub.publish(vel_msg);
}

// 사각형 그리기 서비스 Request에 응답하는 Callback 함수
bool handlepolygon(turtlesim_autodraw::polygon::Request &req, turtlesim_autodraw::polygon::Response &res)
{

	ROS_INFO("[draw_polygon_server] I just got incoming request!");
	
	// vel_msg 메시지 객체 생성
	geometry_msgs::Twist vel_msg;

	// service client로부터 요청된 변 길이 받아오기
	int side_num = req.sidenum;
	ROS_INFO_STREAM("[draw_polygon_server] req.sidelength=" << req.sidelength);

	// service client로부터 요청된 변 길이 받아오기
	float side_length = req.sidelength;
	ROS_INFO_STREAM("[draw_polygon_server] req.sidelength=" << req.sidelength);

	// service client로부터 요청된 반복 회수 받아오기
	int32_t rotations = req.rotations;
	ROS_INFO_STREAM("[draw_polygon_server] req.rotations=" << req.rotations);

	// 현재 한 각도를 몇 번 돌았는지 확인
	// = (요청된 반복 횟수) * side_num
	int current_rotation_outofnum = 0;
	
	while (current_rotation_outofnum < rotations * side_num && ros::ok())
	{
		// 디버그 정보 출력
		ROS_DEBUG_STREAM("[draw_polygon_server] current_rotation_outofnum, rotations=" << current_rotation_outofnum << ", " << rotations);

		// 직선 그리고 회전, 카운터 증가
		draw_in_line(side_length, vel_msg, pub);
		rotate(side_num, vel_msg, pub);
		current_rotation_outofnum++;
	}

	// 여기서 return값에 의미는 없으나 콜백함수 형식을 바꿀 수 없으므로 return 값을 임의로 true로 설정
	return true;
}


int main(int argc, char **argv)
{
	// 노드/노드핸들 초기화
	ros::init(argc, argv, "draw_polygon_server");
	ros::NodeHandle nh;
	ROS_INFO("[draw_polygon_server] Startup");

	// 노드 반복 실행(spin) 주기 설정(ms)
	ros::Rate loop_rate(10);

	// 퍼블리셔 실행 (turtlesim 조종용 토픽 퍼블리시)
	pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ROS_INFO("[draw_polygon_server] Advertising publisher /turtle1/cmd_vel");

	// 서비스 서버 실행 (그릴 사각형 크기와 반복회수 받는 서비스 서버)
	ros::ServiceServer s = nh.advertiseService<turtlesim_autodraw::polygon::Request, turtlesim_autodraw::polygon::Response>("draw_polygon", handlepolygon);
	ROS_INFO("[draw_polygon_server] Advertising service server draw_polygon");

	// CTRL+C 등 종료 명령 받기 위해 ros::spin()대신 ros::ok(), 루프, ros::spinOnce() 조합
	while(ros::ok()) ros::spinOnce();

	return 0;
}