#include<iostream>
#include<bits/stdc++.h> 
#include <set> 
//#include<ros/ros.h>
#include<vector>
#include<map>
#include<string.h>
#include<list>
#include<math.h>
#include "astar.h"
using namespace std; 



vector<Pair> gen_aircraft_obj()
{
    vector<Pair> ret_val;

    int m = 5;
    for(int i=-m; i<m; i++)
    {
        for(int j=-m; j<m; j++)
        {
            for(int k=-m; k<m; k++)
            {
                ret_val.push_back(make_pair(i,make_pair(j,k)));
            }
        }
    }

    ret_val.push_back(make_pair(0,make_pair(0,-3)));

    return ret_val;

}

vector<Pair> init_center()
{
    vector<Pair> temp_center;
    temp_center.push_back(make_pair(0,make_pair(0,0)));
    return temp_center;
}

map<string, vector<Pair> > aircraft_obj = {{"aircraft_points", gen_aircraft_obj()}, {"init_center", init_center()}};

//////////////////////////Configuration Parameters/////////////////////////



vector<Pair> config:: movement_list()
{
    int step_size = 1;
    vector<Pair> move;
    move.push_back(make_pair(step_size, make_pair(0,0)));
    move.push_back(make_pair(-step_size, make_pair(0,0)));
    move.push_back(make_pair(0, make_pair(step_size,0)));
    move.push_back(make_pair(0, make_pair(-step_size,0)));
    move.push_back(make_pair(0, make_pair(0,step_size)));
    move.push_back(make_pair(0, make_pair(0,-step_size)));

    return move;
}
config::config()
{
    move_list =  movement_list();
    z_move_cost = 5;
} 
    

double config::manhattan(Pair p, Pair p_end)
{
    return 10*(abs(p.first - p_end.first) + abs(p.second.first - p_end.second.first) + abs(p.second.second - p_end.second.second));
}

double config::square(Pair p, Pair p_end)
{
    return (abs(p.first - p_end.first)*abs(p.first - p_end.first) +  abs(p.second.first - p_end.second.first)* abs(p.second.first - p_end.second.first) + abs(p.second.second - p_end.second.second)*abs(p.second.second - p_end.second.second));
}

///////////////////End of COnfiguration Parameters //////////////////////////////


/////////////////// Point NODE class begin///////////////////////



Node::Node(){}

Node::Node(Pair ipoint)
{
    point = ipoint;
    //Node parent;
    G = 0;
    H = 0;
}

Pair Node::get_point()
{
    return point;
}

double Node::move_cost(Pair ipoint)
{
    if(point.second.first == ipoint.second.first)
    {
        return 1.0;
    }
    else
    {
        return astar_config.z_move_cost;
    }
}

void Node::print() const
{
    cout<<"point :" << point.first<<" "<<point.second.first<<" "<<point.second.second<< "G: "<< G << "H: "<<H<<"F: "<<G+H<<endl;
}

bool Node::operator< (const Node& e) const 
{
    bool result = true;
    if ((G+H) < (e.G+e.H))
        result = true;
    else if ((e.G,e.H) < (G+H))
        result = false;
}


/* 
bool operator<(const Node& p1, const Node& p2)
{
return ((p1.G + p1.H) < (p2.G + p2.H));
}*/
//////////////////////////NODE class end///////////////////////////////////////
typedef pair<double, Node> npair;
typedef set<Node> sNode;


///////////////////////A_star class begin//////////////////////////////////



A_star::A_star(Pair iend_pos)
{
    end_pos = iend_pos;
    vector<Pair> movement_list = astar_config.move_list;
    for_radius = aircraft_radius(aircraft_obj["aircraft_points"]);
    //cout<<for_radius[0]<<endl;
    horizontal_radius = for_radius[0];
    //cout<<"asd"<<horizontal_radius<<endl;
    vertical_radius = for_radius[1];
}

vector<double> A_star::aircraft_radius(vector<Pair> iaircraft_points)
{
    double horizontal = 0;
    double vertical = 0;
    double r;
    Pair center_point = make_pair(0,make_pair(0,0));
    vector<double> res;
    //res.push_back(7.07107);

    cout<<"center point is :"<<center_point.first<<" "<<center_point.second.first<<" "<<center_point.second.second<<endl;

    for(int i=0; i<iaircraft_points.size(); i++)
    {
        if (iaircraft_points[i].second.second == 0)
        {
            r = distance(iaircraft_points[i], center_point);
            if(horizontal < r)
            {
                horizontal = r;
            }
        }
        if(iaircraft_points[i].second.second < 0)
        {
            r = distance(iaircraft_points[i], center_point);
            if(vertical < r)
            {
                vertical = r;
            }
        }
    }
    cout<<"horizontal radius and vertical: "<<horizontal<<" "<<vertical<<endl;;
    //cout<<"exited  "<<res[0]<<endl;
    res.push_back((double)horizontal);
    res.push_back((double)vertical);
    //cout<<"exited  "<<res[1]<<endl;
    return res;
}


double A_star::distance(Pair pt1, Pair pt2)
{
    return(sqrt(pow((pt1.first - pt2.first),2)+pow((pt1.second.first - pt2.second.first),2)+pow((pt1.second.second - pt2.second.second),2)));
}

bool A_star::success(Node current_obj, bool center_point_can_go)
{
    if(center_point_can_go)
    {
        if (current_obj.get_point() == end_pos)
        {
            return true;
        }
    }
    else
    {
        if(distance(current_obj.get_point(), end_pos) <= (max(horizontal_radius , vertical_radius)))
        {
            return true;
        }
    }
    return false;
}

void A_star::debug_get_openlist_info()
{
    npair first_element = openlist.top();
    Node node = first_element.second;
    openlist.push(make_pair((node.G+node.H), node));
}

vector<Pair> A_star::find_path(Pair start_pos, set<Pair> obstacle_pos_list)
{
    //priority_queue <npair, vector<npair>, greater<npair> > openlist;
    sNode closed;
    Node start_obj(start_pos);

    openlist.push(make_pair((start_obj.G+start_obj.H), start_obj));
    bool center_point_can_go = is_valid(end_pos, obstacle_pos_list);
    vector<Pair> path;
    int count = 0;
    //cout<<"Target Position Found inside Obstacle, Finding Path Failed!"<<endl;
    for (set<Pair>::iterator it=obstacle_pos_list.begin(); it!=obstacle_pos_list.end(); ++it)
    {
        if((*it) == end_pos)
        {
            cout<<"Target Position Found inside Obstacle, Finding Path Failed!"<<endl;
            return path;
        }
    }

    while(!openlist.empty())
    {//cout<<"inside while"<<endl;
        //debug_get_openlist_info();
        count++;
        //cout<<"counting"<<endl;
        Node current_obj = openlist.top().second;

        if(success(current_obj, center_point_can_go))
        {
            while(current_obj.parent)
            {
                path.push_back(current_obj.get_point());
                current_obj = *current_obj.parent;
            }

            path.push_back(current_obj.get_point());
            cout<<"count:"<<count<<endl;
            cout<<"extend"<<openlist.size()+closed.size()<<endl;
            reverse(path.begin(), path.end());
            return path;
        }

        closed.insert(current_obj);
        //cout<<"reached end"<<endl;
        extend_round(current_obj, obstacle_pos_list);
        //cout<<openlist.size()<<" before pop  This is the sixe of open list"<<endl;
        openlist.pop();
        if(count<5)
        {
            priority_queue <npair, vector<npair>, greater<npair> > openlist_copy1 = openlist;
            while(!openlist_copy1.empty())
            {
                //cout<<"in while loop"<<endl;
                npair temp = openlist_copy1.top();
                //temp.second.print();
                //cout<<"in while loop"<<endl;
                print_point(temp.second.point);
                openlist_copy1.pop();
            }
            cout<<"/////////////////////////////////////"<<endl;
        }
    }
    cout<<"Open List Run Out, No Path Found."<<endl;
}

void A_star::print_point(Pair p)
{
    cout<<"point is: "<<p.first<<" "<<p.second.first<<" "<<p.second.second<<endl;
}


void A_star::extend_round(Node current_obj, set<Pair> obstacle_pos_list)
{
    //cout<<"entered extend_round"<<endl;
    vector<Pair> movement_list = astar_config.move_list;
    Pair new_point;
    double new_g;
    //vector<Node> node_obj;
    
    for(int i=0; i<movement_list.size(); i++)
    {   
       
        
        new_point = make_pair(movement_list[i].first + current_obj.get_point().first, make_pair(movement_list[i].second.first + current_obj.get_point().second.first, movement_list[i].second.second + current_obj.get_point().second.second));
        
        //Node node_obj(new_point);
        Node node_obj(new_point);
        //cout<<"after node made"<<endl;
        
        if(!is_valid(new_point, obstacle_pos_list))
        {//cout<<"is_valid"<<endl;
            continue;
        }
        if(is_in_closedlist(new_point))
        {//cout<<"is_in_closedlist"<<endl;
            continue;
        }
        if(is_in_openlist(new_point))
        {//cout<<"is_in_openlist"<<endl;
            new_g = current_obj.G + current_obj.move_cost(new_point);
            if(node_obj.G > new_g)
            {
                node_obj.G = new_g;
                node_obj.parent = &current_obj;
            }
            
        }
        else
        {//cout<<"else is_in_openlist"<<endl;
            //Node node_obj(new_point);
            node_obj.G = current_obj.G + current_obj.move_cost(new_point);
            node_obj.H = astar_config.square(node_obj.get_point(), end_pos);
            node_obj.parent = &current_obj;
            openlist.push(make_pair((node_obj.G+node_obj.H), node_obj));
            
        }
        
        //openlist.top().second.print();
    }
    //cout<<openlist.size()<<" fasle  This is the sixe of open list"<<endl;
}


bool A_star::is_in_closedlist(Pair p)
{
    for (sNode::iterator it=closed.begin(); it!=closed.end(); ++it)
    {
        if((*it).point == p)
        {
           // cout<<"true"<<endl;
            return true;
        }
    }
    //cout<<"false"<<endl;
    return false;
}

bool A_star::is_in_openlist(Pair p)
{
    priority_queue <npair, vector<npair>, greater<npair> > openlist_copy = openlist;
    //cout<<"copy made"<<endl;
    npair temp;

    while(!openlist_copy.empty())
    {
        //cout<<"in while loop"<<endl;
        temp = openlist_copy.top();
        //temp.second.print();
        //cout<<"in while loop"<<endl;
        //print_point(p);
        if(temp.second.point == p)
        {
            //cout<<"true"<<endl;
            //cout<<openlist.size()<<" This is the sixe of open list"<<endl;
            return true;
        }
        openlist_copy.pop();
    }
    //cout<<"fasle"<<endl;
    
    return false;

}

bool A_star::is_valid(Pair pt, set<Pair> iobstacle_pos_list)
{
    set<Pair> intersect;
    set<Pair> obstacle_map_indexed = iobstacle_pos_list;
    set<Pair> aircraft_points;
    
    vector<Pair> item = aircraft_obj["aircraft_points"];
    for(int i=0; i<item.size(); i++)
    {
        aircraft_points.insert(item[i]+pt);
    }
    set_intersection(obstacle_map_indexed.begin(),obstacle_map_indexed.end(),aircraft_points.begin(),aircraft_points.end(),std::inserter(intersect,intersect.begin()));
    if(!intersect.empty())
    {
        //cout<<"fasle"<<endl;
        return false;
    }
    else
    {
        //cout<<"true"<<endl;
        return true;
    }
    
}

bool A_star::path_is_valid(vector<Pair> path_points, set<Pair> iobstacle_pos_list)
{
    for(int i=0; i<path_points.size(); i++)
    {
        if(!is_valid(path_points[i], iobstacle_pos_list))
        {
            cout<<"Path is invalid"<<endl;
            return false;
        }
    }

    cout<<"Path is valid"<<endl;
    return true;
}
/*
int main()
{
    return 0;
}
*/