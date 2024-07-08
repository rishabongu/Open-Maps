/*-------------------------------------------
Program 5: Open Street Maps: a console-based program to input a campus map and navigate two people at two different buildings to meet at the best building
Course: CS 251, Fall 2023, UIC
System: Advanced zyLab
Author: Risha Bongu
------------------------------------------- */
// application.cpp
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <utility>
#include <stack>
#include <limits>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"

using namespace std;
using namespace tinyxml2;

//global variable infinity to refer anytime 
const double INF = numeric_limits<double>::max();

class prioritize 
{
public:
  bool operator()(const pair<long long,double>& p1, const pair<long long, double>& p2) const
  {
      //compares the second element 
      if(p1.second != p2.second)
          return p1.second > p2.second; 
    // if second element equal, then comapre the fist element 
    return p1.first > p2.first; 
  }
};

//helper function for dijkstra's algorithm which finds the shortest path from one location to the other location using the parameters of starting point, the graph, and the distance which stores the calculated distances
map<long long, long long> DijkstraShortestPath(long long start, graph<long long, double>& G, map<long long, double>& distance){
    //stores the vertices and their distances  
    priority_queue< pair<long long,double>,
    vector<pair<long long, double>>,prioritize> unvisitedQueue;  
    //keeps track of the visited vertices
    map<long long, double> visitedNdes;
    //stores the predecessors for each vertex
    map<long long, long long>pred;
    //pushes vertices into the priority queue
    for(auto& vertex: G.getVertices()) {
      distance.insert(make_pair(vertex,INF));
      pred[vertex] = -1;
      unvisitedQueue.push(make_pair(vertex, INF));
   }
   //start has a distance of 0 from itself
   distance[start] = 0;
   unvisitedQueue.push(make_pair(start, 0));
    long long currV;
    double weight = 0;
    double alternativePathDistance = 0;
    while (!unvisitedQueue.empty()) {
      //visits vertex with minimum distance from start
        currV = unvisitedQueue.top().first;
        unvisitedQueue.pop();
         //checks if the current vertex has been visited
        if (distance.at(currV) == INF) {
            continue; 
        }
        else if (visitedNdes.find(currV) != visitedNdes.end()){
            continue;
        }
        //shows current vertex has been visited 
        else{
            visitedNdes[currV] = 1;
        }
        for(auto& adjV: G.neighbors(currV)) {
            G.getWeight(currV,adjV,weight);
            alternativePathDistance = distance.at(currV) + weight;  
         //if shorter path  found updates adjV's distance and predecessor
            if (alternativePathDistance < distance.at(adjV)) {
                distance.at(adjV) = alternativePathDistance;
                unvisitedQueue.push(make_pair(adjV, alternativePathDistance)); 
                pred.at(adjV) = currV;
            
            }
        }
    }
   return pred;
}

//helper function to get the path from point A to point B using predecessor array and returning a vector showing the path from start to end vertex in order
void getPath(map<long long, long long>& predecessors, long long endVertex) {
    vector<long long> path;
    stack<long long> pathStack;
    //builds the path by traversing the predecessors
    long long currV = endVertex;
    while (predecessors[currV] != -1) {
        pathStack.push(currV);
        currV = predecessors[currV];
    }
    //pushes the starting vertex onto the stack 
    pathStack.push(currV);
    //prints the path 
    cout << "Path: ";
    while (!pathStack.empty()) {
        currV = pathStack.top();
        pathStack.pop();
        cout << currV << "->";
    }
    cout << endl;

}

//helper fucntion that finds the of two buildings and finds the building closest to that center building uisng the two buildings information and coordinates
void locateCenterBuilding(vector<BuildingInfo>& Buildings, BuildingInfo building1, BuildingInfo building2, Coordinates &cord3 ){
    //finds the midpoint on the two buildings 
    Coordinates midpoint = centerBetween2Points(building1.Coords.Lat,building1.Coords.Lon,building2.Coords.Lat,building2.Coords.Lon);

    double minDistance = INF;
    BuildingInfo destinationBuilding;

    for(const auto&building: Buildings){
        //calcuates the distance of the center building to closest building 
        double distance = distBetween2Points(midpoint.Lat, midpoint.Lon,building.Coords.Lat, building.Coords.Lon);
        //closest building information 
        if(distance < minDistance){
            minDistance = distance;
            destinationBuilding = building;
            cord3 = destinationBuilding.Coords;
        }
    }
    //prints out the destination building and the coordinates 
     cout << "Destination Building:" << endl;
     cout <<  " " << destinationBuilding.Fullname << endl;
     cout << " (" << destinationBuilding.Coords.Lat << ", " << destinationBuilding.Coords.Lon << ")" << endl;
}

//helper fucntion that searches the footways for the nearest start and nearest destination nodes wiht the use of coordinates 
void locateNearestNodes(vector<FootwayInfo>& Footways, map<long long, Coordinates>& Nodes, long long &findNode, Coordinates &nodeCords){

    double minDistance = INF;

    for(const auto&path: Footways){
        for(const auto&nodes: path.Nodes){
            //calculates the distance between the node and the building 
             double distance = distBetween2Points(nodeCords.Lat, nodeCords.Lon, Nodes[nodes].Lat, Nodes[nodes].Lon);
             if(distance < minDistance){
                minDistance = distance;
                findNode = nodes;
            }
        }  
        
    }

}

//helper fucntion for searching the buildings on the map that user inputs as start building and destiantion builinding using building and coordinate information 
void searchBuildings(vector<BuildingInfo>& Buildings, string givenOne, string givenTwo, BuildingInfo &buildingOne, BuildingInfo &buildingTwo, bool &found, Coordinates &cord1, Coordinates &cord2 ){

    string name;
    string name2;

    bool found1 = false;
    bool found2 = false;
    
     for(BuildingInfo& building: Buildings){
          //checks if the abbrevation matches the input of the first building
         if(building.Abbrev == givenOne){
             name = building.Fullname;
             cord1 = building.Coords;
             found1 = true;
             buildingOne = building;
         }
         //checks if the abbrevation matches the input of the second building 
         if(building.Abbrev == givenTwo){
             name2 = building.Fullname;
             cord2 = building.Coords;
             found2 = true;
             buildingTwo = building;
         }
 
     }
     //if the searching the building through abbreviation did not find the building then the substring search is used 
      for(BuildingInfo& building: Buildings){
          //checks if the building name has the same name as input 
        if (found1 == false && building.Fullname.find(givenOne) != string::npos){
            name = building.Fullname;
            cord1 = building.Coords;
            found1 = true;
            buildingOne = building;
        }
        //checks if the second building name has the same name as input 
        if (found2 == false && building.Fullname.find(givenTwo) != string::npos){
            name2 = building.Fullname;
            cord2 = building.Coords;
            found2 = true;
            buildingTwo = building;

        }
      }
      //if building not found then prints the error messages 
      if(found1 != true){
          cout << "Person 1’s building not found" << endl;
          found = false; 

      }
      else if(found2 != true){
          cout << "Person 2’s building not found" << endl;
            found = false; 
      }
      //if building found, then prints the two building names and coordinates 
      else{
          found = true; 
          cout << endl;
          cout << "Person 1's point:" << endl;
          cout <<  " " << name << endl;
          cout << " (" << cord1.Lat << ", " << cord1.Lon << ")" << endl;
    
          cout << "Person 2's point:" << endl;
          cout << " " << name2 << endl;
          cout << " (" << cord2.Lat << ", " << cord2.Lon  << ")" << endl;
      }
}


void application(
    map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
    vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
    string person1Building, person2Building;


    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    while (person1Building != "#") {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);
        BuildingInfo buildingOne, buildingTwo ;
    
        bool found;
        long long node1 = -1;
        long long node2 = -1;
        long long node3 = -1;

        Coordinates cord1;
        Coordinates cord2;
        Coordinates cord3;

        //fucntion call for seach building 
        searchBuildings(Buildings, person1Building, person2Building , buildingOne, buildingTwo, found, cord1, cord2 );
        //finds the center building, nearest nodes, and prints if buildings are found
        if(found != false){
            //fucntion call to get the center building 
            locateCenterBuilding(Buildings,buildingOne,buildingTwo,cord3);
            //fucntion call 3 times to get the nearest node 
            locateNearestNodes(Footways,Nodes,node1,cord1);
            locateNearestNodes(Footways,Nodes,node2,cord2);
            locateNearestNodes(Footways,Nodes,node3,cord3);
            //prints the nearest node information 
            cout << endl;
            cout << "Nearest P1 node:" << endl;
            cout <<  " " << node1 << endl;
            cout << " (" << Nodes[node1].Lat << ", " <<  Nodes[node1].Lon << ")" << endl;
            cout << "Nearest P2 node:" << endl;
            cout <<  " " << node2 << endl;
            cout << " (" << Nodes[node2].Lat << ", " <<  Nodes[node2].Lon << ")" << endl;
            cout << "Nearest destination node:" << endl;
            cout <<  " " << node3 << endl;
            cout << " (" << Nodes[node3].Lat << ", " <<  Nodes[node3].Lon << ")" << endl;
            cout << endl;


    while(true){

        map<long long, long long> pred1;
        map<long long, double> distance1;

        map<long long, long long> pred2;
        map<long long, double> distance2;

        long long tempNode2  = node2;
        long long tempNode1  = node1;
        
        //calls the dijkstra's algorithm to get the shortest path for person 1 and person 2 
        pred1 = DijkstraShortestPath(tempNode1, G,  distance1);
        pred2 = DijkstraShortestPath(tempNode2, G,  distance2);

        double dist1 = distance1.at(node3);
        double dist2 = distance2.at(node3);

        //checks if node 2 is reachable and prints message 
        if (distance1[node2] == INF){
            cout << endl;
            cout << "Sorry, destination unreachable." << endl;
            break;
        }
        //prints out the person 1 and 2 information of path and miles
        else if (distance1[node3] != INF && distance2[node3] != INF){
            cout << "Person 1's distance to dest: " << dist1 << " miles" << endl;
            getPath(pred1,node3);
            cout << "Person 2's distance to dest: " << dist2 << " miles" << endl;
            getPath(pred2,node3);
            break;
        }
        //checks that there is a valid path from Building 1 and Building 2 to the center building and prints out message
        else{
            cout << endl;
            cout << "At least one person was unable to reach the destination building. Finding next closest building..." << endl;
            break;
        
            }
        }


    }
   
    cout << endl; 
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }    
}

//helper fucntion to add the vertices to the graph 
void addVerticesToGraph( graph<long long, double> &G, map<long long, Coordinates> &Nodes){
  for(const auto& node: Nodes){
     G.addVertex(node.first);//add the vertices
  }
}

//helper fucntion to add the edges to the graph based on footways 
void addEdgesToGraph(vector<FootwayInfo>&Footways,graph<long long, double> &G, map<long long, Coordinates> &Nodes){
   for(const auto& node: Footways){
    for(int i = 0; i < node.Nodes.size() -1; ++i){
      long long node1 = node.Nodes[i];
      long long node2 = node.Nodes[i+1];
      //calculates the distance between two nodes
      double distance = distBetween2Points(Nodes[node1].Lat,Nodes[node1].Lon, Nodes[node2].Lat,Nodes[node2].Lon);
        //adds the edges 
        G.addEdge(node1,node2,distance);
        G.addEdge(node2,node1,distance);
    }
  }
}


int main() {
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //fucntion call to add vertices
  addVerticesToGraph(G, Nodes);
  //fucntion call to add edges
  addEdgesToGraph(Footways,G,Nodes);


  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
