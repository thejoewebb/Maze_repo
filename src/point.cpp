#include <point.h>
#include <maze.h>
#include "solve_corridor.cpp"

using std::min;
using std::max;

namespace ecn
{

Maze Point::maze;    

// final print
// maze deals with the color, just tell the points
void Point::print(const Point &parent) const
{
    Position startpoint = Position(x,y);
    Position endpoint = Position(parent.x,parent.y);
    //initialise x and y increments
    int x_incr(0), y_incr(0);
    //initialise corridor boolean
    bool corridor = false;

    // if parent is in line with child in x direction
    if(x == parent.x){
        y_incr = y - parent.y > 0 ? -1 : 1;

        //check direct route for walls
        for(int i=y; i!=parent.y; i+= y_incr){
            if(!maze.isFree(x,i)){
                //if walls found, break and set corridor to true
                corridor = true;
                break;
            }
        }
    //if parent is in line with child in y direction
    }else if(y == parent.y){
        x_incr = x - parent.x > 0 ? -1 : 1;

        //check direct route for walls
        for(int j=x; j!=parent.x; j+=x_incr){
            if(!maze.isFree(j,y)){
                //if walls found break and set corridor to true
                corridor = true;
                break;
            }
        }
    //if parent is not in line with child
    }else
        corridor = true;

    //if path from child to parent is not straight line
    if(corridor){

        //set start and end point of the path
        //Position startpoint = Position(x,y);
        //Position endpoint = Position(parent);
        //initialise vector of nodes and set best node as dummy position (0,0)
        std::vector<Position> nodes;
        Position best_node(0,0);

        nodes = startpoint.parents(endpoint);
        best_node = nodes[0];

        //if more than one node are the same as parent, find the one with shortest path to child
        if(nodes.size()>1)
            for(auto node: nodes)
                if(node.distToParent()<best_node.distToParent())
                    best_node = node;

        //pass through each point on path from parent to child
        //maze.passThrough(x,y);
        for(auto cell: best_node.path)
            maze.passThrough(cell.x,cell.y);

    //parent is in line with child so carry out pass through script normally
    }else{
//        int x_incr(0), y_incr(0);

//        if(x - parent.x)
//            x_incr = x - parent.x > 0 ? -1 : 1;
//        else
//            y_incr = y - parent.y > 0 ? -1 : 1;
        int k = 1;

        while(x + k*x_incr != parent.x || y + k*y_incr != parent.y)
        {
            maze.passThrough(x + k*x_incr,
                             y + k*y_incr);
            k++;
        }
        maze.passThrough(parent.x,parent.y);
    }

}

void Point::start()
{
    maze.write(x, y);
}

// online print, color depends on closed / open set
void Point::show(bool closed, const Point & parent)
{
    //set blue and red point and x and y increment
    const int b = closed?255:0, r = closed?0:255;
    Position startpoint = Position(x,y);
    Position endpoint = Position(parent.x,parent.y);
    int x_incr(0), y_incr(0);

    //initialise corridor bool
    bool corridor = false;

    // if parent is in line with child in x direction
    if(x == parent.x){
        y_incr = y - parent.y > 0 ? -1 : 1;
        //check direct route for walls
        for(int i=y; i!=parent.y; i+= y_incr){
            if(!maze.isFree(x,i)){
                //if walls found break and set corridor to true
                corridor = true;
                break;
            }
        }
    // if parent is in line with child in y direction
    }else if(y == parent.y){
        x_incr = x - parent.x > 0 ? -1 : 1;
        //check direct route for walls
        for(int j=x; j!=parent.x; j+=x_incr){
            if(!maze.isFree(j,y)){
                //if walls found break and set corridor to true
                corridor = true;
                break;
            }
        }
    //if parent is not in line with child
    }else
        corridor = true;

    //if path from child to parent is not straight line
    if(corridor){

        //set start and end point of the path
        //Position startpoint = Position(x,y);
        Position endpoint = Position(parent);
        //initialise vector of nodes and set best node as dummy position (0,0)
        std::vector<Position> nodes;
        Position best_node(0,0);

        //iterate through nodes returned by custom function parents()
        for(auto node: startpoint.parents(endpoint))
            if(node==endpoint)
                //add all nodes which = goal node (parent)
                nodes.push_back(node);

        best_node = nodes[0];

        //if more than one node are the same as parent, find the one with shortest path to child
        if(nodes.size()>1)
            for(auto node: nodes)
                if(node.distToParent()<best_node.distToParent())
                    best_node = node;

        //pass through each point on path from child to parent
        maze.write(x,y,r,0,b,false);
        for(auto cell: best_node.path)
            maze.write(cell.x,cell.y,r,0,b,false);

    //if parent is in line with child write using regular script
    }else{

        if(x != parent.x)
            for(int i = min(x, parent.x); i <= max(x, parent.x);++i)
                maze.write(i, y, r, 0, b, false);
        else
            for(int j = min(y, parent.y); j <= max(y, parent.y);++j)
                maze.write(x, j, r, 0, b, false);

        maze.write(x, y, r, 0, b);
    }

}

}
