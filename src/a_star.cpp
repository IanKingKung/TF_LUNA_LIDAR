#include "a_star.h"

std::vector<POINT2D> a_star(const POINT2D& start, const POINT2D& goal, const GRID& grid) {
    std::vector<POINT2D> path;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;  //greater<Node> sets max heap to min heap

    //start with start point, expand until we reach goal point
    Node start_node;
    start_node.position = start;
    start_node.g_score = 0.0f;
    start_node.h_score = 0.0f; 
    start_node.f_score = start_node.g_score + start_node.h_score;
    start_node.parent = nullptr;

    open_set.push(start_node);
    while(!open_set.empty()) {  //more nodes to explore
        Node current_node = open_set.top();
        open_set.pop();

        // If we reached the goal, reconstruct the path by following parent pointers & then reverse list
        if (current_node.position == goal) {
            const Node* curr = &current_node;
            while (curr->parent != nullptr) {
                path.push_back(curr->position);
                curr = curr->parent;
            }
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

            float tentative_g_score = current_node.g_score + 0.05f; // uniform cost to move to a neighbor

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
            neighbor_node.parent = new Node(current_node); // Store parent for path reconstruction


            open_set.push(neighbor_node);
        }
    }

    return path;
}