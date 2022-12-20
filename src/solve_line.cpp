#include <a_star.h>
#include <maze.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
    typedef std::unique_ptr<Position> PositionPtr;

    int distance{0};

public:
    // constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    //constructor with distance
    Position(int _x, int _y, int _distance) : Point(_x, _y){

        distance = _distance;
    }

    int distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return distance;
    }

    std::vector<Position> scan(int _x,int _y)
    {
        std::vector<pair<int,int>> check = {{-1,0},{1,0},{0,-1},{0,1}};
        std::vector<Position> directions;

        //check all adjacent cells for wall. if the cell is free, add it to the list of adjacent free cells
        for(auto [i,j]: check)
            if(maze.isFree(_x+i,_y+j))
                directions.push_back(Position(_x+i,_y+j));

        //return list of free directions. If the length of the vector !=2 the scan cell is either a dead end or junction
        return directions;

    }

    std::vector<PositionPtr> children(){
        //initialise vector of children
        std::vector<PositionPtr> generated;

        //check all free adjacent cells
        for(auto cell: scan(x,y))

            //progress in y direction
            if(cell.x == x){

                //set iteration direction
                auto dir = cell.y - y;
                auto y_new = cell.y;
                //iterate until wall ahead or free cell to side
                while(maze.isFree(x,y_new+dir) && !maze.isFree(x-1,y_new) && !maze.isFree(x+1,y_new))
                    y_new += dir;

                //add child to vector
                generated.push_back(std::make_unique<Position>(x,y_new, abs(y_new-y)));

            }else{

                //set iteration direction
                auto dir = cell.x - x;
                auto x_new = cell.x;
                //iterate until wall ahead or free cell to side
                while(maze.isFree(x_new + dir,y) && !maze.isFree(x_new,y-1) && !maze.isFree(x_new,y+1))
                    x_new += dir;

                //add child vector
                generated.push_back(std::make_unique<Position>(x_new,y, abs(x_new-x)));
            }
        return generated;
    }
};

int main( int argc, char **argv )
{
    // load file
    std::string filename = "../mazes/maze.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Position::maze.load(filename);

    // initial and goal positions as Position's
    Position start = Position::maze.start(),
             goal = Position::maze.end();

    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("cell");
    cv::waitKey(0);

}
