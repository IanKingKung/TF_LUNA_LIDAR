#include "a_star.h"

struct PointHash {  //custom hash function for POINT2D
    std::size_t operator()(const POINT2D& p) const {
        std::size_t h1 = std::hash<float>{}(p.x);
        std::size_t h2 = std::hash<float>{}(p.y);
        return h1 ^ (h2 << 1); // Combine the two hash values
    }
};

std::vector<POINT2D> a_star(const POINT2D& start, const POINT2D& goal, const GRID& grid) {
    std::vector<POINT2D> path;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;  //greater<Node> sets max heap to min heap
    std::unordered_map<POINT2D, POINT2D, PointHash> came_from;   //keep track of parent nodes for path reconstruction
    std::unordered_map<POINT2D, float, PointHash> g_scores;  //cost from start to node

    //start with start point, expand until we reach goal point
    Node start_node;
    start_node.position = start;
    start_node.g_score = 0.0f;
    start_node.h_score = 0.0f; 
    start_node.f_score = start_node.g_score + start_node.h_score;
    open_set.push(start_node);
    g_scores[start] = 0.0f;
    

    while(!open_set.empty()) {  //more nodes to explore
        Node current_node = open_set.top();
        open_set.pop();

        // If we reached the goal, reconstruct the path by following parent pointers & then reverse list
        if (current_node.position == goal) {
            POINT2D curr_point = current_node.position;
            while (curr_point != start) {
                path.push_back(curr_point);
                curr_point = came_from[curr_point]; // move to parent node
            }
            path.push_back(start); // add the start point to the path
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explore the 8 neighboring cells 
        std::vector<POINT2D> neighbors = {
            {current_node.position.x, current_node.position.y + 1}, // Up
            {current_node.position.x, current_node.position.y - 1}, // Down
            {current_node.position.x - 1, current_node.position.y}, // Left
            {current_node.position.x + 1, current_node.position.y},  // Right
            {current_node.position.x - 1, current_node.position.y - 1}, // Down-left
            {current_node.position.x + 1, current_node.position.y - 1}, // Down-right
            {current_node.position.x - 1, current_node.position.y + 1}, // Up-left
            {current_node.position.x + 1, current_node.position.y + 1}  // Up-right
        };

        for (const auto& neighbor_pos : neighbors) {
            if (!grid.is_within_bounds(neighbor_pos) || grid.is_obstacle(neighbor_pos)) {
                continue; // Skip out-of-bounds or obstacle cells
            }

            float move_cost = (neighbor_pos.x != current_node.position.x && neighbor_pos.y != current_node.position.y) ? 0.05f * sqrt(2.0f) : 0.05f; // Diagonal vs straight cost
            float tentative_g_score = current_node.g_score + move_cost; 

            //only add Nodes if we haven't seen this node before or this path is better than any previous path
            if(g_scores.find(neighbor_pos) == g_scores.end() || tentative_g_score < g_scores[neighbor_pos]) {
                g_scores[neighbor_pos] = tentative_g_score;

                //calculate h (heuristic) score
                float dx = abs(neighbor_pos.x - goal.x);
                float dy = abs(neighbor_pos.y - goal.y);
                float D = 0.05f; //length of each node in m --> keep grid resolution consistent with meters
                float D2 = D * sqrt(2.0f);  //diagonal distance

                Node neighbor_node;
                neighbor_node.position = neighbor_pos;
                neighbor_node.g_score = tentative_g_score;
                neighbor_node.h_score = D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy); //octile distance heuristic
                neighbor_node.f_score = neighbor_node.g_score + neighbor_node.h_score;

                came_from[neighbor_node.position] = current_node.position;  //set parent of neighbor to current node
                open_set.push(neighbor_node);
            }
        }
    }

    return path;
}