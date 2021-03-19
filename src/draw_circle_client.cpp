#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

// srv폴더에 만든 서비스 파일에서 생성되는 헤더파일로 catkin_make 실행 중 생성된다.
#include "turtlesim_autodraw/circle.h"


int main(int argc, char **argv)
{
	// 노드/노드핸들 초기화
	ros::init(argc, argv, "draw_circle_client");
	ros::NodeHandle nh;
	ROS_INFO("[draw_circle_client] Startup");

	// 서비스 서버가 켜질 때까지 대기한다.
	// rospy.wait_for_service('draw_circle')

	try
	{
		// 입력받은 문자열 버퍼
		std::string inputString;

		// 반지름 길이 받아오기
		std::cout << "Enter Radius : ";
		std::cin.clear(); // 기존에 혹시 모를 처리되지 않은 입력값을 삭제, 입력값 오류 방지
		std::getline(std::cin, inputString); // 엔터키 칠 때까지 받음
		double radius = std::stod(inputString); // 문자열로 받은 입력값을 double형으로 변경
		if (radius <= 0)  // 잘못된 길이가 입력되었을 경우 실행 중단, 에러 출력 후 강제종료
			throw "radius <= 0";
		ROS_DEBUG_STREAM("radius = " << radius);

		// 문자열 버퍼 비우기(재활용)
		inputString = "";

		// 반복 횟수 받아오기
		std::cout << "Enter number of rotations : ";
		std::cin.clear(); // 기존에 혹시모를 처리되지 않은 입력값을 삭제, 입력값 오류 방지
		std::getline(std::cin, inputString); // 엔터키 칠 때까지 받음
		int rotations = std::stoi(inputString); // 문자열로 받은 입력값을 int형으로 변경
		if (rotations <= 0)						// 잘못된 횟수가 입력되었을 경우 실행 중단, 에러 출력 후 강제종료
			throw "rotations <= 0";
		ROS_DEBUG_STREAM("rotations = " << rotations);

		// 서비스 클라이언트 연결
		turtlesim_autodraw::circle svccircle;
		ros::ServiceClient svccircleClient = nh.serviceClient<turtlesim_autodraw::circle>("draw_circle");
		// 서비스 서버로 요청값(radius과 side_length, rotations) 저장
		svccircle.request.radius = radius;
		svccircle.request.rotations = rotations;
		// 저장된 요청값 전송(실행)
		svccircleClient.call(svccircle);
	}
	catch (const ros::Exception e)
	{
		ROS_ERROR_STREAM("Service call failed: " << e.what());
	}
	catch (const char* s)
	{
		ROS_ERROR_STREAM(s);
	}
}