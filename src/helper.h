//
// Created by Ayush J on 10/22/2019.
//


#ifndef WP_GEN2PIT_HELPER_H
#define WP_GEN2PIT_HELPER_H

#include <iostream>
#include <vector>
#include <math.h>

using namespace std;


struct coordinate
{
    int x;
    int y;
    coordinate(int x_,int y_):x(x_ ),y(y_ ){}
    coordinate(): x(0), y(0){ };

    void set_coordinate(int a, int b){
        x = (a);
        y = (b);
    }

};

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

class helper {
public:
 //   vector<int> seg;
//    std::vector<vector<int>> map ;
//    struct coordinate  robot_position =  coordinate(10,0);
//    struct coordinate pit_centre =  coordinate(10,20);
//    struct coordinate2  dir_vec =  coordinate2();
//    double min_step = 0.5;
 //   vector<coordinate> list_wp;

    helper(){}

    coordinate generate_next_wp(coordinate  curr_pos){}

    int signn(double a){}

    void dir_vec_update(){}

    bool edge_reached(coordinate pos, int side_shift) { }

    bool isTraversible(coordinate vec){}

    coordinate get_traversible_left(coordinate vecc){ }

    coordinate get_traversible_right(coordinate vec){}

    bool func(){}

};


#endif //WP_GEN2PIT_HELPER_H
