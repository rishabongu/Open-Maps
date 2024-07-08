/*-------------------------------------------
Program 5: Open Street Maps: graph class that uses an adjacency list representation 
Course: CS 251, Fall 2023, UIC
System: Advanced zyLab
Author: Risha Bongu
------------------------------------------- */
///graph.h
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <unordered_map>
#include <list>
#include <map>
#include <algorithm>

using namespace std;
//class graph 
template<typename VertexT, typename WeightT>
class graph {
    private:
        //data structure that stores the nodes and weight
         unordered_map<VertexT,unordered_map<VertexT,WeightT>>graphNode;
         //stores the number of edges 
         int numEdges; 


 public:
  // constructor:
  // constructs an empty graph where n is the max number of vertices
  // the graph to contains
  graph() {
    numEdges = 0;
  }

  // NumVertices:
  // returns the number of vertices currently in the graph.
  int NumVertices() const {
    return graphNode.size(); 
  }

  // NumEdges:
  // returns the number of edges currently in the graph.
  int NumEdges() const {
    return numEdges;
  }

  // addVertex:
  // adds the vertex v to the graph and returns true 
  // if the graph is full or the vertex already exists in the graph
  // returns false 
  bool addVertex(VertexT v) {
    //checks if graph is full or vertex exists and returns false
    if(graphNode.count(v)){
        return false;
    }
    //assigns an empty map for new vertex
    unordered_map<VertexT,WeightT>emptyMap;
    //adds the vertex to the graph 
    graphNode[v] = emptyMap;

    return true; 
  }

  // addEdge:
  // adds the edge (from, to, weight) to the graph and returns true 
  // if the vertices do not exist or
  // graph is full, return false.
  // if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
      //checks if both vertices exist in the graph
    if(!graphNode.count(from) || !graphNode.count(to)  ){
        return false;
    }
    //checks if edge exists 
    else if (!graphNode.at(from).count(to)){
        numEdges++;
    }
    //updates the edge weight
    graphNode[from][to] = weight;
    return true;

  }

  // getWeight: returns the weight with a given edge.
  // returns the weight if the edge exists and true is returned
  // if the edge does not exist then false is returned
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    //checks if source vertex exists and returns false
    if (!graphNode.count(from))
        return false;
    //checks if destination vertex exists and returns false
    if (!graphNode.count(to))
        return false;
    
    unordered_map<VertexT,WeightT>elements = graphNode.at(from);
    //checks if edge exists
    if(!elements.count(to)){
        return false;
    }
    //gets the weight 
    weight = graphNode.at(from).at(to);

    return true;
  }

  // neighbors:
  // returns a set containing the neighbors of all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT>  S;
    //checks if vertex exsists
    if(!graphNode.count(v)){
        return S;
    }
    //gets the neighbors from the adjacency list
   const unordered_map<VertexT, WeightT>& neighborsMap = graphNode.at(v);
    for(const auto& neighbor : neighborsMap){
        //gets the neighboring vertex
        VertexT neighborVertex = neighbor.first;
        //insterts 
        S.insert(neighborVertex);

    }
    return S;
  }

  // getVertices:
  // returns a vector with all the vertices currently in the graph
  vector<VertexT> getVertices() const {
    vector<VertexT> vertices;
    //goes through each entry in the graph
    for(const auto & entry : graphNode){
        //adds the vertex to the graph 
        vertices.push_back(entry.first);
    }
    //returns the vertices
    return vertices;
  }

  //                      
  // dump
  // dumps the internal state of the graph for debugging purposes
  //properly output the vertices and edges, based final implementation
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    for (const auto& entry : graphNode) {
        output << " " << entry.first << endl; // prints the vertices
    }

    output << endl;
    output << "**Edges:" << endl;

    for (const auto& entry : graphNode) {
        VertexT from = entry.first;
        const unordered_map<VertexT, WeightT>& neighborsMap = entry.second;

        for (const auto& neighbor : neighborsMap) {
            VertexT to = neighbor.first;
            WeightT weight = neighbor.second;
            //prints the edges 
            output << from <<  ": (" << from << "," << to << "," <<  weight << ") ";
        }

      output << endl;
    }
    output << "**************************************************" << endl;
  }
};