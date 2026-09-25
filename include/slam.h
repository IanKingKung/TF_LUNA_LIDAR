#ifndef SLAM_H
#define SLAM_H

#include <vector>

struct PoseNode {
    int id;
    float x, y, theta;
};

namespace Slam {    //A namespace lets you group your variables and functions together globally i.e. SLAM::run_slame()
    // 'extern' tells the compiler these live in the .cpp file, 
    extern std::vector<PoseNode> pose_graph;
    extern std::vector<float> information_vector;   //size 3 * num_nodes
    extern std::vector<std::vector<float>> information_matrix;  //size 3N * 3N

    void update_measurements(); //update information matrix
    bool update_status(); //estimate optimal robot/landmark positions
    void run_slam();
    void add_pose_node(float x, float y, float theta);
}

#endif // SLAM_H