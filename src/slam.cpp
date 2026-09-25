#include "slam.h"

void Slam::add_pose_node(float x, float y, float theta) {
    int new_id = pose_graph.size();
    pose_graph.push_back({new_id, x, y, theta});
}

void SLAM::update_measurements() {
    if (pose.graph.size() < 2) return;

    const PoseNode prev = pose_graph[pose_graph.size() - 2];
    const PoseNode curr = pose_graph[pose_graph.size() - 1];

    float dx = prev.x - curr.x;
    float dy = prev.y - curr.y;
    float dtheta = prev.theta - curr.theta;

    //each node takes up a 3x3 square in the matrix due to three dimensions
    int i = prev.id;
    int j = curr.id;

    int idx_i = i * 3;
    int idx_j = j * 3;

    //weights for position and theta --> higher weights indicating higher confidence
    float w_pos = 100.0f;
    float w_theta = 50.0f;

    //make sure that information matrix and vector are required size
    int required_size = (std::max(i,j) + 1) * 3;
    if(information.size() < required.size) {
        information_matrix.resize(required_size, std::vector<float>(required_size, 0.0f));
        for (auto& row : information_matrix) {
                row.resize(required_size, 0.0f);
            }
        information_vector.resize(required_size, 0.0f);
    }

    for (int k = 0; k < 3; ++k) {
            int row_i = idx_i + k;
            int row_j = idx_j + k;
            float weight = (k == 2) ? w_theta : w_pos;
            float measurement = (k == 0) ? dx : (k == 1) ? dy : dtheta;

            // Diagonal blocks (Self-connections)
            information_matrix[row_i][row_i] += weight;
            information_matrix[row_j][row_j] += weight;

            // Off-diagonal blocks (Cross-connections between prev and curr)
            information_matrix[row_i][row_j] -= weight;
            information_matrix[row_j][row_i] -= weight;

            // Information vector contributions (b vector)
            information_vector[row_i] -= measurement * weight;
            information_vector[row_j] += measurement * weight;
    }
}

void SLAM::update_status() {
    int num_nodes = pose_graph.size();
    if (num_nodes == 0) return true;

    // Step A: Resize/Reset information matrix (3N x 3N) and vector (3N)
    int size = num_nodes * 3;
    information_matrix.assign(size, std::vector<float>(size, 0.0f));
    information_vector.assign(size, 0.0f);

    // Step B: Populate H and b from your graph constraints (Linearization)
    // ... (accumulate Jacobians and information weights) ...

    // Step C: Solve H * delta_x = b (e.g., using Gauss-Newton or a solver)
    float max_correction = 0.0f;
    // std::vector<float> delta_x = solve_linear_system(information_matrix, information_vector);
    // for(int i = 0; i < size; ++i) {
    //     apply correction to pose_graph coordinates...
    //     max_correction = std::max(max_correction, std::abs(delta_x[i]));
    // }

    // Step D: Check Convergence (Stop updating if changes are negligible)
    float convergence_tolerance = 0.001f; // 1 millimeter threshold
    if (max_correction < convergence_tolerance) {
        return true; // Converged! We can stop optimizing for now.
    }

    return false; // Still optimizing
}

void Slam::run_slam() {
    update_measurements();

    int max_iterations = 0;
    for (int i = 0; i < max_iterations; i++) {
        bool converged = update_status();
        if (converged) break;   //stop updates when map is settled
    }
}