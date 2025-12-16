#include <limits>
#include <memory>
#include <algorithm>

#include "ShortestPathHeuristic.h"


ShortestPathHeuristic::ShortestPathHeuristic(size_t source, size_t graph_size, const AdjacencyMatrix &adj_matrix) 
    : source(source), all_nodes(graph_size+1, nullptr) {
    size_t num_of_objectives = adj_matrix.get_num_of_objectives();
    size_t i = 0;
    for (auto node_iter = this->all_nodes.begin(); node_iter != this->all_nodes.end(); node_iter++) {

        std::vector<double> h_vector(num_of_objectives-1, 0);
        h_vector.push_back(MAX_COST);
        *node_iter = std::make_shared<Node>(i++, std::vector<double>(num_of_objectives, 0), h_vector);

    }

    compute(num_of_objectives-1, adj_matrix); 
    
}


std::vector<double> ShortestPathHeuristic::operator()(size_t node_id) { //the length heuristic value is allways in the last element of the vector
    return this->all_nodes[node_id]->h;
}


// Implements Dijkstra shortest path algorithm per cost_idx cost function
void ShortestPathHeuristic::compute(size_t cost_idx, const AdjacencyMatrix &adj_matrix) {
    // Init all heuristics to MAX_COST
    for (auto node_iter = this->all_nodes.begin(); node_iter != this->all_nodes.end(); node_iter++) {
        (*node_iter)->h[cost_idx] = MAX_COST;
    }

    NodePtr node;
    NodePtr next;

    // Init open heap
    Node::more_than_specific_heurisitic_cost more_than(cost_idx);
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    this->all_nodes[this->source]->h[cost_idx] = 0;
    open.push_back(this->all_nodes[this->source]);
    std::push_heap(open.begin(), open.end(), more_than);


    while (open.empty() == false) {
        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            next = this->all_nodes[p_edge->target];

            // Dominance check
            if (next->h[cost_idx] <= (node->h[cost_idx]+p_edge->cost[cost_idx])) {
                continue;
            }

            // If not dominated push to queue
            next->h[cost_idx] = node->h[cost_idx] + p_edge->cost[cost_idx];
            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);
        }
    }
}
