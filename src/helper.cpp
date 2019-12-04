//#include "helper.h"
#include <iostream>
#include "ros/ros.h"
#include <vector>
#include <fstream>
#include <sstream>
// #include <opencv2/opencv.hpp>
#include <math.h>
#include <string>
#include <stdlib.h>
using namespace std;

#define MAP_FILE "/home/hash/catkin_ws/src/waypoint_pit_planner/src/mapp.csv" 
#define PI 3.14159265359
#define MIN_STEP_GENERAL 1
#define MIN_STEP_TO_EDGE 0.75

// string MAP_FILE = "/home/hash/catkin_ws/src/waypoint_pit_planner/src/mapp.csv";

//---------------------------------------------------------------------------------------
//----------------------------stores exact cordinaties - Used ---------------------------
//---------------------------------------------------------------------------------------
// struct coordinate
// {
//     int x;
//     int y;
//     coordinate(int x_,int y_):x(x_ ),y(y_ ){}
//     coordinate(): x(0), y(0){ };

//     void set_coordinat(int a, int b){
//         x = (a);
//         y = (b);
//     }

// };

struct coordinate
{
    double x;
    double y;
    coordinate(double x_,double y_):x(x_ ),y(y_ ){}
    coordinate(): x(0), y(0){ };

    void set_coordinate(double a, double b){
        x = (a);
        y = (b);
    }

};

//--------------------------------------------------------------------------------
//--------------------------------struct for helper-------------------------------
//--------------------------------------------------------------------------------
struct coordinate2
{
    double x;
    double y;
    coordinate2(double x_,double y_):x(x_ ),y(y_ ){}
    coordinate2(): x(0), y(0){ };
    void set_coordinate(double a, double b){
        x = (a);
        y = (b);
    }
};

//--------------------------------------------------------------------------------
//--------------------------------struct to compute-------------------------------
//--------------------------------------------------------------------------------
class helper {
private:
	bool reached_edge_flag = false;

public:
    // vector<int> seg;
    std::vector<vector<int> > map =  convert_csv_to_vector(MAP_FILE);
    struct coordinate  robot_position =  coordinate(10,0);
    struct coordinate pit_centre =  coordinate(1300,1300);
    struct coordinate2  dir_vec =  coordinate2();
    double min_step = MIN_STEP_GENERAL; //resolution of the map
    vector<coordinate> list_wp;
    double dir_yaw;
    //string MAP_FILE = "/home/hash/catkin_ws/src/waypoint_pit_planner/src/mapp.csv";

    double get_direction_vec(){

        double i = -PI/2 +(( atan2(dir_vec.y, dir_vec.x)));
        if (i<0){
            i += 2*PI;
        }
        return i;

    }

    helper(){
	
	//MAP_FILE  = fname;
        // seg = vector<int>(10,1);
        // vector<int> seg2 = vector<int>(10,0);
        // seg.insert(seg.end(), seg2.begin(), seg2.end());
        // map = convert_csv_to_vector(MAP_FILE);//generate_map_vec();
        //vector<vector<int> >(20, seg);
        // map[10][5] = 0;
        // map[10][6] = 0;
        // map[9][6] = 0;
        // map[9][5] = 0;
        // map[12][11] = 1;
        // map[12][10] = 1;
        // map[12][12] = 1;
        // map[8][11] = 1;
        // map[8][10] = 1;
        // map[8][12] = 1;
    }
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

std::vector<int> split(const std::string& s, char delimiter)
{
    std::vector<int> result;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        // cout<<token<<endl;

        result.push_back(stod(token));
    }
    return result;
}

vector<vector<int> > convert_csv_to_vector(const string &file_name)
{
    std::ifstream file(file_name);
    string line;
    string number;
    string temp;
    vector<vector<int> > map;
    int count=0;

    while (getline(file, line,'\n'))
    {
        auto res = split(line,',');
        map.push_back(res);
        // cout<<"res:" <<res.size()<<endl;
    }
    // for (int j = 0 ; j <3; j++){
    // for (int i = 0; i<50;i++){
    // cout<<map[0][i];
    // }
    // cout<<endl;
    // }
    return (map);
}



//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

    void set_location(const int x, const int y){
      robot_position.x = x;
      robot_position.y = y;
      return;
    }


//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

    coordinate generate_next_wp(coordinate  curr_pos){
        struct coordinate vec;
        vec.set_coordinate((int)(curr_pos.x + min_step * dir_vec.x) ,(int)(curr_pos.y + min_step * dir_vec.y) ); //was using ceil when coordinate was int
        return vec;
    }
//

coordinate generate_next_wp_edge_checker(coordinate  curr_pos, string caller){

        struct coordinate vec;
        // cout << caller << endl;
        // cout << "Direction vector " << dir_vec.x << " " <<dir_vec.y << endl;
        vec.set_coordinate((int)(curr_pos.x + signn(dir_vec.x)) ,(int)(curr_pos.y + signn(dir_vec.y)) ); //was using ceil when coordinate was int
        return vec;

    }


//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

    bool get_reached_edge_status(){
    	return this->reached_edge_flag;
    }

//--------------------------------------------------------------------------------
//--------------------------------sign() function---------------------------------
//--------------------------------------------------------------------------------
    int signn(double a){
        if (a>0)
        {
            return 1;
        }
        else if (a<0)
        {
            return -1;
        }
        else
        {
        	return 1;
        }
    }

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
    void dir_vec_update(){

        dir_vec.x = pit_centre.x - robot_position.x;
        dir_vec.y = pit_centre.y - robot_position.y;
        double tmp = sqrt(pow(dir_vec.x,2) + pow(dir_vec.y,2));
        dir_vec.x/= tmp;
        dir_vec.y/=tmp;
        
    }

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

    bool edge_reached(coordinate pos, int side_shift) { //true means could not reach
        // cout<<"---------------------------------------------------------------"<<endl;
        int count = 0;
        struct coordinate  vec2;
        vec2 = generate_next_wp_edge_checker(pos, "edge_reached");
        for (int i = 0; i <= 7; i++) {
            cout<<"Inside edge Reached "<< (int)vec2.x<<" " <<(int)vec2.y<< " " <<map[(int)vec2.y][(int)vec2.x] << endl;
            if (map[(int)vec2.x][(int)vec2.y] != 1) {
                count++;
            }
            // cout<<" counter  : "<< count<<endl;
            vec2 = generate_next_wp_edge_checker(vec2, "edge_reached");
        }
        if(side_shift >3){
        	this->reached_edge_flag = true;
            return true;
        }
        if (count>4){
            // if(edge_reached(coordinate(pos.x , (pos.y-signn(dir_vec.y))), side_shift+1)
            //    && edge_reached(coordinate(pos.x , (pos.y+signn(dir_vec.y))),side_shift+1)){
            //    	this->reached_edge_flag = true;
                this->reached_edge_flag = true;
                return true;
            // }
        }
        return false;
    }
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------


    void update_min_step(coordinate pos) { //true means could not reach
        int x = (int)pos.x;
        int y = (int)pos.y;
        this->min_step = MIN_STEP_GENERAL;
        for(int i = -2; i<=2;i++){
            for(int j = -2;j <=2; j++){
                if( x + i <map[0].size() && x + i>=0){
                    if(y + j <map.size() && y + j>=0){
                        if (map[x + i][y + j] == 0){
                            this->min_step = MIN_STEP_TO_EDGE;
                        }
                    }
                }
            }
        }
    }
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

    bool isTraversible(coordinate vec){
        if (map[(int)vec.x][(int)vec.y] == 0){
            return false;
        }
        return true;
    }

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

    coordinate get_traversible_left(coordinate vecc){
        int count = 0;
        coordinate vec_originial = vecc;
        while(!isTraversible(vecc) && count < 5){
            if(dir_vec.x > dir_vec.y)
                vecc = coordinate( (int)(vecc.x ), (int)(vecc.y-this->min_step*(dir_vec.y)));
            else
                vecc = coordinate( (int)(vecc.x-this->min_step*(dir_vec.x)),  (int)(vecc.y));
            count++;
        }
        if (count == 5){
            return vec_originial;
        }
        return vecc;
    }

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

    coordinate get_traversible_right(coordinate vec){
        int count = 0;
        struct coordinate vec_originial = vec;
        while(!isTraversible(vec)){
            if(dir_vec.x > dir_vec.y)
                vec = coordinate( (int)(vec.x ), (int)(vec.y+ this->min_step*(dir_vec.y)));

            else
                vec = coordinate( (int)(vec.x+this->min_step*(dir_vec.x)),  (int)(vec.y));
            count++;
           if(count > 5){
               break;
           }
        }
        if (count == 5){
            return vec_originial;

        }
        return vec;
    }

//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

    bool func(){
        struct coordinate vec;
        dir_vec_update();
        while (!edge_reached(robot_position,0)){
             dir_vec_update();
             update_min_step(robot_position);
             vec = generate_next_wp( robot_position);
             if (isTraversible(vec)){
                 robot_position = vec;
                 list_wp.push_back(vec);
                 cout<<"Going Straight"<<endl;
                 return true;
             }
             else{
                 coordinate vec_left = get_traversible_left(vec);
                 if (vec_left.x == vec.x && vec_left.y == vec.y )
	                {
						coordinate vec_right = get_traversible_right(vec);
						if (vec_right.x == vec.x && vec_right.y == vec.y ){
						 	return false;
						}
						else
						{
							 vec = vec_right;
							 robot_position = vec;
							 list_wp.push_back(vec);
                             cout<<"Going Right"<<endl;
							 return true;
						}
	                }
                 else{
                     vec = vec_left;
                     robot_position = vec;
                     list_wp.push_back(vec);
                     cout<<"Going Left"<<endl;
                     return true;
                 }
             }
        }
        cout<<"--------------------------"<<"EDGE REACHED"<<endl;
        return false;

    }

};
