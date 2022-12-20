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
    std::vector<Position> path;
    // constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    //constructor with distance
    Position(int _x, int _y, int _distance) : Point(_x, _y){

        distance = _distance;
    }
    //constructor with path
    Position(int _x, int _y, std::vector<Position> _path) : Point(_x, _y){
        distance = _path.size();
        path = _path;
    }

    int distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return distance;
    }

    bool isCorridor(const Position parent){

        bool corridor = false;
        int y_incr, x_incr;
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

        return corridor;

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

    std::vector<PositionPtr> children()
    {
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> generated;
        //generated.push_back(std::make_unique<Position>(x,y));
        // suppress std::cout from maze.end()
        std::cout.setstate(std::ios_base::failbit);
        //find goal node
        Position goal = Position::maze.end();
        //reset std::cout suppression
        std::cout.clear();

        //for each free cell adjacent to current cell (up to 4 directions)
        for(auto cell: scan(x,y)){
            //initialise vector to record path
            std::vector<Position> path;
            //add first adjacent cell
            path.push_back(cell);
            //set previous cell for direction detection
            Pair cell_previous = Pair(x,y);

            //if scan(x,y)=2 then cell x,y is a corridor
            //if cell = end cell automatically end while and add end cell to children
            while(scan(cell.x,cell.y).size() == 2 && (cell.x != goal.x || cell.y != goal.y)){

                auto cells = scan(cell.x,cell.y);

                //check against previous cell to ensure no looping
                //have to compare as pair, errors when comparing positions
                if(Pair(cells[0].x,cells[0].y) != cell_previous){
                    cell_previous = Pair(cell.x,cell.y);
                    cell=Position(cells[0].x,cells[0].y);
                    path.push_back(cell);

                }else{
                    cell_previous = Pair(cell.x,cell.y);
                    cell=Position(cells[1].x,cells[1].y);
                    path.push_back(cell);
                }

            }

            //add children (points that = target or are at junction/dead end)
            generated.push_back(std::make_unique<Position>(cell.x,cell.y,path));

        }

        return generated;
    }

    std::vector<Position> parents(const Position goal)
    {
        // this method should return path to goal reachable from this one
        std::vector<Position> generated;

        //for each free cell adjacent to current cell (up to 4 directions)
        for(auto cell: scan(x,y)){
            //initialise vector to record path
            std::vector<Position> path;
            //add first cell
            path.push_back(cell);
            Pair cell_previous = Pair(x,y);

                //if scan(x,y)=2 then cell x,y is a corridor
                //if cell = end cell automatically end while and add end cell to children
                while(scan(cell.x,cell.y).size() == 2 && (cell.x != goal.x || cell.y != goal.y)){

                    auto cells = scan(cell.x,cell.y);

                    //check against previous cell to ensure no looping
                    //have to compare as pair, errors when comparing positions
                    if(Pair(cells[0].x,cells[0].y) != cell_previous){
                        cell_previous = Pair(cell.x,cell.y);
                        cell=Position(cells[0].x,cells[0].y);
                        path.push_back(cell);

                    }else{
                        cell_previous = Pair(cell.x,cell.y);
                        cell=Position(cells[1].x,cells[1].y);
                        path.push_back(cell);
                    }

                }

                //add children (points that = target or are at junction/dead end)
                if(cell.x == goal.x && cell.y == goal.y)
                    generated.push_back(Position(cell.x,cell.y,path));
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
