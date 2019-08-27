#ifndef ASTAR_H
#define ASTAR_H

#include<iostream>
#include<bits/stdc++.h> 
#include <set> 
//#include<ros/ros.h>
#include<vector>
#include<map>
#include<string.h>
#include<list>
#include<math.h>

using namespace std;

typedef pair<double,  pair<double, double> > Pair; 
typedef pair<string, Pair> sPair;
typedef set <Pair> setpair;

template <typename T,typename U, typename R>                                                   
std::pair<T,pair<U,R> > operator+(const std::pair<T,pair<U,R> > & l,const std::pair<T,pair<U,R> > & r)
{   
    return {l.first+r.first,{l.second.first+r.second.first, l.second.second+r.second.second}};                                                                       
}


vector<Pair> gen_aircraft_obj();
vector<Pair> init_center();

class config
{
    public:
    vector<Pair> move_list;
    double z_move_cost;


    vector<Pair> movement_list();
    config();

    double manhattan(Pair p, Pair p_end);
    double square(Pair p, Pair p_end);
};

class Node
{
    public:
    Pair point;
    double G;
    double H;
    config astar_config; 
    Node* parent;

    Node();

    Node(Pair ipoint);

    Pair get_point();

    double move_cost(Pair ipoint);

    void print() const;

    bool operator< (const Node& e) const;
};

typedef pair<double, Node> npair;
typedef set<Node> sNode;

class A_star
{
    public:
    Pair end_pos;
    priority_queue <npair, vector<npair>, greater<npair> > openlist;
    sNode closed;
    config astar_config;
    double horizontal_radius;
    double vertical_radius;
    vector<double> for_radius;
    vector<Pair> path;


    A_star(Pair iend_pos);

    vector<double> aircraft_radius(vector<Pair> iaircraft_points);

    double distance(Pair pt1, Pair pt2);

    bool success(Node current_obj, bool center_point_can_go);
 
    void debug_get_openlist_info();

    vector<Pair> find_path(Pair start_pos, set<Pair> obstacle_pos_list);


    void extend_round(Node current_obj, set<Pair> obstacle_pos_list);

    void print_point(Pair p);
    bool is_in_closedlist(Pair p);
    
    bool is_in_openlist(Pair p);
    bool is_valid(Pair pt, set<Pair> iobstacle_pos_list);

    bool path_is_valid(vector<Pair> path_points, set<Pair> iobstacle_pos_list);
}; 

#endif