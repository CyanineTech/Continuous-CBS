#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <vector>
#include "tinyxml2.h"
#include "const.h"
#include "structs.h"

class Map
{
private:
    std::vector<std::vector<int>> grid;
    std::vector<gNode> nodes;
    std::vector<std::vector<Node>> valid_moves;
    int  height, width, size;
    int  connectedness;
    double agent_size;
    bool map_is_roadmap;
    bool check_line(int x1, int y1, int x2, int y2);
    bool get_grid(const char* FileName);
    bool get_roadmap(const char* FileName);
public:
    const std::vector<gNode>& get_nodes() const { return nodes; }
    const std::vector<std::vector<Node>>& get_valid_moves() const { return valid_moves; }
    
    void remove_edge(int node_a, int node_b) {
        if (node_a >= 0 && node_a < (int)valid_moves.size()) {
            auto& moves = valid_moves[node_a];
            moves.erase(std::remove_if(moves.begin(), moves.end(),
                       [node_b](const Node& n) { return n.id == node_b; }),
                       moves.end());
        }
        if (node_b >= 0 && node_b < (int)valid_moves.size()) {
            auto& moves = valid_moves[node_b];
            moves.erase(std::remove_if(moves.begin(), moves.end(),
                       [node_a](const Node& n) { return n.id == node_a; }),
                       moves.end());
        }
    }
    void remove_directed_edge(int from, int to) {
        if (from >= 0 && from < (int)valid_moves.size()) {
            auto& moves = valid_moves[from];
            moves.erase(std::remove_if(moves.begin(), moves.end(),
                       [to](const Node& n) { return n.id == to; }),
                       moves.end());
        }
    }    
    Map(){agent_size=0.1; connectedness=2;}
    Map(double size, int k){ agent_size = size; connectedness = k; }
    ~Map(){}
    int  get_size() const { return size; }
    bool get_map(const char* FileName);
    bool is_roadmap() const {return map_is_roadmap;}
    bool cell_is_obstacle(int i, int j) const;
    int  get_width() const {return width;}
    gNode get_gNode(int id) const {if(id < int(nodes.size())) return nodes[id]; return gNode();}
    int  get_id(int i, int j) const;
    double get_i (int id) const;
    double get_j (int id) const;
    std::vector<Node> get_valid_moves(int id) const;
    void print_map();
    void printPPM();
};

#endif // MAP_H
