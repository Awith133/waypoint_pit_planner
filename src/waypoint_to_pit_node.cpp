#include "ros/ros.h"
#include "helper.cpp"
#include <vector>
#include <iostream>
#include <waypoint_to_pit/waypoints.h>

using namespace std;

/* The way point genenerator works as a service
Each waypoint is generated in the neighborhood of the robot
*/



bool g_wp(waypoint_to_pit::waypoints::Request &req, waypoint_to_pit::waypoints::Response &res){
	helper test;
    res.wp_received = test.func();
    res.mission_flag = test.get_reached_edge_status();
	res.wp_received = true;
	if (res.wp_received ){
		res.x = test.list_wp[test.list_wp.size() -1 ].x;
		res.y = test.list_wp[test.list_wp.size() -1 ].y;
		return false;
	}
	else if (res.mission_flag){
		return true;
	}
	return false;
}



int main(int argc, char **argv){

	ros::init(argc,argv, "waypoint_to_pit_server");

	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("gen_wp2pit", g_wp);
	ros::spin();

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
