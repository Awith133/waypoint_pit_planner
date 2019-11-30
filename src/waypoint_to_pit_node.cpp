#include "ros/ros.h"
#include "helper.cpp"
#include <vector>
#include <iostream>
#include "geometry_msgs/PolygonStamped.h"
#include <waypoint_pit_planner/waypoints.h>

using namespace std;

/* The way point genenerator works as a service
*/

#define RESOLUTION = 0.5;
#define OFFSET = 0.25

		static double robot_x ;
		static double robot_y ;
		bool odom_received = false;




void getODOM(const geometry_msgs::PolygonStamped::ConstPtr& msg){
			
			double sum_x =0;
			double sum_y = 0;
			int n = msg->polygon.points.size();

			for (int i=0;i<n;i++)
			{
				sum_x += msg->polygon.points[i].x;
				sum_y += msg->polygon.points[i].y;
			}
			//flipped from rviz to image
			robot_y = (sum_x/n )/0.5; 
			robot_x = -1 * (sum_y/n )/0.5;
			odom_received = true;
}


bool g_wp(waypoint_pit_planner::waypoints::Request &req, waypoint_pit_planner::waypoints::Response &res){
// 			// ros::NodeHandle n;
			helper test;
			if (!odom_received){
				return false;
			}
			
			test.set_location(robot_x,robot_y);
			// cout<<robot_x<<robot_y<<endl;
				// cout<<test.func()<<"func"<<endl;
		    res.wp_received = test.func();
		    res.mission_flag = test.get_reached_edge_status();
			cout << "Waypoints Generated: "<< test.list_wp.size() << "Mission Flag: " << res.mission_flag <<endl;
			
				//res.wp_received = true;
			if (res.mission_flag){
				// res.x = 0;
				// res.y = 0;
				return true;}
			if (res.wp_received ){
				res.yaw = test.get_direction_vec();
				res.y = (-1 * (.5*test.list_wp[test.list_wp.size() -1 ].x))-0.25;
				res.x = (.5*test.list_wp[test.list_wp.size() -1 ].y)+.25;
				cout<<"Robot Position "<<robot_x<<"  "<<robot_y<<endl;
				cout<<"Direction Vector " <<test.dir_vec.x<<" "<<test.dir_vec.y<<endl;
				cout<<"Image Point Generated "<<  test.list_wp[test.list_wp.size() -1 ].x << " " << test.list_wp[test.list_wp.size() -1 ].y <<endl;
				cout<<"Way Point to edge Generated "<<  res.x << " " << res.y <<endl;
				return true;
			}
			else if (res.mission_flag){
				// res.x = 0;
				// res.y = 0;
				return true;
			}
			return false;


}


int main(int argc, char **argv){

	ros::init(argc,argv, "waypoint_pit_planner_server");
		ros::Subscriber sub;
		ros::ServiceServer service;
		ros::NodeHandle n;
			sub = n.subscribe("/move_base/local_costmap/footprint", 10,getODOM);
					service = n.advertiseService("gen_wp2pit", g_wp);
	// WPGEN tmp;
	ros::spin();
	// tmp.run();


	return 0;
}






// #include <iostream>
// #include "helper.h"

// using namespace std;
// int main() {
//     helper test;
//     bool reply = test.func();
//     for(auto elem:test.list_wp){
//         cout<<elem.x<< " " <<elem.y<<endl;
//     }
//     vector<vector<char>> map_char;// = vector<>(20, );

//     vector<char> seg = vector<char>(20,'1');
//     map_char = vector<vector<char>>(20,seg );

//     for(int i = 0; i<map_char.size();i++){
//         for(int j = 0; j<map_char[0].size();j++){
//             map_char[i][j] =  (test.map[i][j]) == 0?'0':'1';
//         }
//     }

//     for(auto elem:test.list_wp){

//             map_char[elem.x][elem.y] = '@';
//     }

//     for(int i = 0; i<map_char.size();i++){
//         for(int j = 0; j<map_char[0].size();j++){
//             cout<<" "<<map_char[i][j] << " ";
//         }
//         cout<<endl;
//     }

// }
