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

    void corridor(int _x, int _y, int dir, std::string axis, std::vector<PositionPtr> &vector)
    {
        int i{0};

        while(true){
            if(axis == "x"){

                auto x_new = _x+i*dir;

                if(!maze.isFree(x_new,_y)){
                    if(i>0)
                        vector.push_back(std::make_unique<Position>(_x+(i-1)*dir,_y, i));
                        //corridor = false;
                 return;

                }else if(maze.isFree(_x+i*dir,_y+1) || maze.isFree(_x+i*dir,_y-1)){//|| !maze.isFree(_x+i*dir,_y))
                    vector.push_back(std::make_unique<Position>(_x+i*dir,_y,i+1));
                    //corridor = false;
                    return;
                }

            }else if(axis == "y"){

                if(!maze.isFree(_x,_y+i*dir)){
                    if(i>0)
                        vector.push_back(std::make_unique<Position>(_x,_y+(i-1)*dir, i));
                        //corridor = false;
                    return;

                }else if(maze.isFree(_x+1,_y+i*dir) || maze.isFree(_x-1,_y+i*dir)){// || !maze.isFree(_x,_y+i*dir))
                    ;
                    //corridor = false;
                    return;
                }
            }
            i++;
        }
    }

    //function that returns all free adjacent cells
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
        //add current cell for history check
        //generated.push_back(std::make_unique<Position>(x,y));


        // TODO add free reachable positions from this point
        //for each free cell adjacent to current cell (up to 4 directions)
        for(auto cell: scan(x,y)){
            //initialise vector to record path
            std::vector<Pair> path;
            //add first adjacent cell
            path.push_back(Pair(cell.x,cell.y));
            Pair cell_previous = Pair(x,y);

                //if scan(x,y)=2 then cell x,y is a corridor
                //if cell = end cell automatically end while and add end cell to children
                while(scan(cell.x,cell.y).size() == 2 && (cell.x != maze.end().x || cell.y != maze.end().y)){

                    auto cells = scan(cell.x,cell.y);

                    //if(std::count(path.begin(), path.end(), Pair(cells[0].x,cells[0].y)) == 0){
                    if(Pair(cells[0].x,cells[0].y) != cell_previous){
                        //std::cout<<std::count(path.begin(), path.end(), Pair(cells[0].x,cells[0].y));
                        path.push_back(Pair(cells[0].x,cells[0].y));
                        cell_previous = Pair(cell.x,cell.y);
                        cell=Position(cells[0].x,cells[0].y);


                    }else{
                        //std::cout<<std::count(path.begin(), path.end(), Pair(cells[1].x,cells[1].y));
                        path.push_back(Pair(cells[1].x,cells[1].y));
                        cell_previous = Pair(cell.x,cell.y);
                        cell=Position(cells[1].x,cells[1].y);

                    }


                }

            //add children (points that = target or are at junction/dead end)
            //std::cout<<cell.x<<cell.y<<std::endl;
            generated.push_back(std::make_unique<Position>(cell.x,cell.y,path.size()));

        }
        //remove 1st entry
        //generated.erase(generated.begin());

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
