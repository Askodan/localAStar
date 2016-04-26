#include "node.h"
#include "MakeMarker.h"
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <cmath>
#include <ctime>

using namespace std;

//directions
static int dx[dir] = {1, 1, 0, -1, -1, -1, 0, 1};
int dy[dir] = {0, 1, 1, 1, 0, -1, -1, -1};


string pathFind(int xStart, int yStart, int xFinish, int yFinish, myMap* map)
{
    priority_queue<Node> pq[2]; // list of open (not-yet-tried) Nodes
    int pqi; // pq index
    Node* n0;
    Node* m0;
    int i = 0, j = 0, x = 0, y = 0, xdx = 0, ydy = 0;
    char c;

    int xSize = map->GetXSize();
    int ySize = map->GetYSize();
    int closed_nodes_map[xSize][ySize];
    int open_nodes_map[xSize][ySize];
    int dir_map[xSize][ySize];
    int dir = 8;

    printf("Map details: size: (%d, %d), goal position: %d, %d \n", xSize, ySize, xFinish, yFinish);

    pqi=0;

    // reset the Node maps
    for (y = 0; y < ySize; y++)
    {
        for (x = 0; x < xSize; x++)
        {
            closed_nodes_map[x][y] = 0;
            open_nodes_map[x][y] = 0;
        }
    }

    // create the start Node and push into list of open Nodes
    n0 = new Node(xStart, yStart, 0, 0);
    n0->UpdatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);

    open_nodes_map[x][y] = n0->GetPriority(); // mark it on the open Nodes map

    // A* search
    while (!pq[pqi].empty())
    {
        // Get the current Node w/ the highest priority
        // from the list of open Nodes
        n0 = new Node( pq[pqi].top().GetxPos(), pq[pqi].top().GetyPos(),
                     pq[pqi].top().GetLevel(), pq[pqi].top().GetPriority());

        x = n0->GetxPos(); y=n0->GetyPos();

        pq[pqi].pop(); // remove the Node from the open list
        open_nodes_map[x][y] = 0;
        // mark it on the closed Nodes map
        closed_nodes_map[x][y] = 1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if (x == xFinish && y == yFinish)
        {
            printf("Path found\n");

            // generate the path from finish to start
            // by following the directions
            string path = "";
            while (!(x == xStart && y == yStart))
            {
                //printf("Map value: %d\n", map->GetMap(x, y));
                j = dir_map[x][y];
                c = '0'+ (j + dir / 2) % dir;
                path = c + path;

                x += dx[j];
                y += dy[j];
            }

            //delete unknown places
            printf("Path length: %d\n", path.length());
            for (int i = 0; i < path.length(); ++i) {
                printf("Map value: %d\n",map->GetMap(x, y));
                if(map->GetMap(x, y) == 200) {
                    printf("Erasing path at %d\n", i);
                    path.erase(i);
                    break;
                }

                j = dir_map[x][y];
                x += dx[j];
                y += dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover Nodes
            while(!pq[pqi].empty()) pq[pqi].pop();

            std::cout << "PATH FOUND " << path << '\n' << std::endl;
            return path;
        }

        // generate moves (child Nodes) in all possible directions
        for (i = 0; i < dir; i++)
        {
            xdx = x + dx[i]; ydy = y + dy[i];

            if (!(xdx < 0 || xdx > xSize - 1 || ydy < 0 || ydy > ySize - 1 || map->GetMap(xdx, ydy ) > 200
                || closed_nodes_map[xdx][ydy] == 1))
            {
                // generate a child Node
                m0 = new Node( xdx, ydy, n0->GetLevel(), n0->GetPriority());
                m0->NextLevel(i);
                m0->UpdatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if (open_nodes_map[xdx][ydy] == 0)
                {
                    open_nodes_map[xdx][ydy] = m0->GetPriority();
                    pq[pqi].push(*m0);
                    // mark its parent Node direction
                    dir_map[xdx][ydy] = (i + dir / 2) % dir;
                }
                else if (open_nodes_map[xdx][ydy] > m0->GetPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy] = m0->GetPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy] = (i + dir / 2) % dir;

                    // replace the Node
                    // by emptying one pq to the other one
                    // except the Node to be replaced will be ignored
                    // and the new Node will be pushed in instead
                    while (!(pq[pqi].top().GetxPos() == xdx &&
                           pq[pqi].top().GetyPos() == ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted Node

                    // empty the larger size pq to the smaller one
                    if (pq[pqi].size() > pq[1-pqi].size()) pqi = 1-pqi;
                    while (!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi= 1 - pqi;
                    pq[pqi].push(*m0); // add the better Node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    printf("No route found! \n");
    return ""; // no route found
}

// Determine priority (in the priority queue)
bool operator<(const Node &a, const Node &b)
{
    return a.GetPriority() > b.GetPriority();
}

void VisualizePath(std::string route, myMap *map, ros::Publisher pub){
    // follow the route on the map and display it
    if(route.length()>0)
    {
        int j; char c;

        ///start point
        int sizeX = map->GetXSize();
        int x = 0;
        int y = sizeX + 1;

        for (int i = 0; i < route.length(); i++)
        {
            c = route.at(i);
            j = atoi(&c);
            x = x + dx[j];
            y = y + dy[j];

            Position<float> p = map->getPos(sizeX * y + x);
            makeMarker(pub, i + 10000, p.x, p.y, (float)0, (float)1, (float)1, (float)1);
        }
    }
}
