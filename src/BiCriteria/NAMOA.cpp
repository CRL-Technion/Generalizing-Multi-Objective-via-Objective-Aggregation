#include "NAMOA.h"
#include "Utils/MapQueue.h"
#include <iostream>

// Helper functions for dominance checking (OriginalMO mode)
static bool is_dominated_dr(NodePtr node, std::list<NodePtr>& list, EPS eps){
    for (auto n: list){
        if (is_dominated_dr(node, n, eps)){
            return true;
        }
    }
    return false;
}

static void add_node_dr(NodePtr node, std::list<NodePtr>& list){
    for (auto it = list.begin(); it != list.end(); ){
        if (is_dominated_dr((*it), node)){
            it = list.erase(it);
        } else {
            it ++;
        }
    }
    list.push_back(node);
}

static bool is_dominated_dr(NodePtr node, std::list<NodePtr>& list){
    for (auto n: list){
        if (is_dominated_dr(node, n)){
            return true;
        }
    }
    return false;
}

// Helper function for dominance checking (MOWithAggregation mode)
// Returns true if node is dominated by any vector in list
// vec dominates node if vec[i] <= node->g[i] for all i (vec is better or equal on all objectives)
static bool is_dominated(NodePtr node, std::vector<std::vector<double>>& list){
    for (const auto& vec : list){
        bool is_dominate = true;
        for (size_t i = 0; i < node->g.size(); i++){
            if (vec[i] > node->g[i]){
                is_dominate = false;
                break;
            }
        }
        if (is_dominate){
            return true;
        }
    }
    return false;
}

// Constructor
NAMOAdr::NAMOAdr(const AdjacencyMatrix &adj_matrix, EPS eps, NAMOAdrMode mode, const LoggerPtr logger)
    : AbstractSolver(adj_matrix, eps, logger), mode(mode) {}

// Get solver name
std::string NAMOAdr::get_solver_name() {
    if (mode == NAMOAdrMode::ORIGINAL_MO) {
        return "NAMOAdr-OriginalMO";
    } else {
        return "NAMOAdr-MOWithAggregation";
    }
}

// Compute heuristic based on mode
std::vector<double> NAMOAdr::compute_heuristic(const Heuristic& heuristic, size_t node_id) {
    if (mode == NAMOAdrMode::ORIGINAL_MO) {
        // Original mode: return full heuristic vector
        return heuristic(node_id);
    } else {
        // Aggregation mode: return simplified vector {0, h[last]}
        auto h = heuristic(node_id);
        return std::vector<double>{0, h[adj_matrix.get_num_of_objectives() - 1]};
    }
}

// Create initial node based on mode
NodePtr NAMOAdr::create_initial_node(size_t source, const Heuristic& heuristic) {
    if (mode == NAMOAdrMode::ORIGINAL_MO) {
        // Original mode: full g and h vectors
        return std::make_shared<Node>(
            source,
            std::vector<double>(adj_matrix.get_num_of_objectives(), 0),
            heuristic(source)
        );
    } else {
        // Aggregation mode: full g vector, simplified h vector
        std::vector<double> h_s = heuristic(source);
        return std::make_shared<Node>(
            source,
            std::vector<double>(adj_matrix.get_num_of_objectives(), 0),
            std::vector<double>{0, h_s[adj_matrix.get_num_of_objectives() - 1]}
        );
    }
}

// Main search algorithm
void NAMOAdr::operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit) {
    this->start_logging(source, target);
    auto start_time = std::clock();

    // Clear edge tracking from any previous search
    counted_edges.clear();

    NodePtr node;
    NodePtr next;

    NodeQueue open(this->adj_matrix.size()+1);

    bool is_aggregation = (mode == NAMOAdrMode::MO_WITH_AGGREGATION);

    // Different storage strategies based on mode
    std::vector<double> storage_aggregation;  // For MO_WITH_AGGREGATION - stores best last objective value
    std::vector<std::vector<std::vector<double>>> closed_vector;  // For MO_WITH_AGGREGATION - stores all g-vectors
    std::vector<std::list<NodePtr>> storage_original;  // For ORIGINAL_MO - stores and prunes NodePtrs

    if (is_aggregation) {
        storage_aggregation = std::vector<double>(this->adj_matrix.size()+1, MAX_COST);
        closed_vector = std::vector<std::vector<std::vector<double>>>(this->adj_matrix.size()+1);
    } else {
        storage_original = std::vector<std::list<NodePtr>>(this->adj_matrix.size()+1);
    }

    // Create initial node
    node = create_initial_node(source, heuristic);
    open.insert(node);

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            this->end_logging(solutions, false);
            std::cout << "\n----- Time limit reached -----\n" << std::endl;
            return;
        }

        // Pop min from queue and process
        node = open.pop();
        num_generation +=1;

        // Dominance check - differs by mode
        if (is_aggregation) {
            // MOWithAggregation dominance check
            if ((((1+this->eps[1])*node->f[1]) >= storage_aggregation[target]) ||
                is_dominated(node, closed_vector[node->id])) {
                continue;
            }
            closed_vector[node->id].push_back(node->g);
            storage_aggregation[node->id] = node->g.back();
        } else {
            // OriginalMO dominance check
            if (is_dominated_dr(node, storage_original[target], this->eps) ||
                is_dominated_dr(node, storage_original[node->id])) {
                continue;
            }
            add_node_dr(node, storage_original[node->id]);
        }

        num_expansion += 1;

        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {

            // Edge detection time tracking - differs by mode
            if (!is_aggregation) {
                // OriginalMO: track edge detection time here
                // Use canonical edge representation (min, max) to ensure each physical edge
                // is counted exactly once, regardless of which direction we traverse it first.
                // For example, edge 5→7 and edge 7→5 both map to canonical pair (5,7).
                auto canonical_edge = std::minmax(node->id, p_edge->target);
                if (counted_edges.insert(canonical_edge).second) {
                    // insert() returns pair<iterator, bool> where .second is true if element was newly inserted
                    total_edge_detection_time = total_edge_detection_time + p_edge->detection_doration;
                }
            }

            size_t next_id = p_edge->target;

            // Compute next_g: max for all objectives except last, sum for last
            std::vector<double> next_g(node->g.size());
            for (size_t i = 0; i < next_g.size()-1; i++){
                next_g[i] = std::max( node->g[i], p_edge->cost[i]);
            }
            next_g[next_g.size()-1] = node->g[next_g.size()-1] + p_edge->cost[next_g.size()-1];

            // Compute next_h using mode-specific logic
            auto next_h = compute_heuristic(heuristic, next_id);

            next = std::make_shared<Node>(next_id, next_g, next_h, node);

            // Dominance check before insertion - differs by mode
            if (is_aggregation) {
                // MOWithAggregation dominance check
                // Quick check on last objective only - try to reject nodes as fast as possible
                // before computing edge_detection_time. Full dominance check comes later.
                if ((((1+this->eps[1])*(next_g.back()+next_h[1])) >= storage_aggregation[target])) {
                    continue;
                }
            } else {
                // OriginalMO dominance check
                if (is_dominated_dr(next, storage_original[next_id]) ||
                    is_dominated_dr(next, storage_original[target], this->eps)) {
                    continue;
                }
            }

            // Edge detection time tracking - MOWithAggregation does it here (after dominance check)
            // Small optimization: dominance check filters out many edges before we need to compute their safety/risk vector
            if (is_aggregation) {
                // Use canonical edge representation (min, max) to ensure each physical edge
                // is counted exactly once, regardless of which direction we traverse it first.
                // For example, edge 5→7 and edge 7→5 both map to canonical pair (5,7).
                auto canonical_edge = std::minmax(node->id, next_id);
                if (counted_edges.insert(canonical_edge).second) {
                    // insert() returns pair<iterator, bool> where .second is true if element was newly inserted
                    total_edge_detection_time = total_edge_detection_time + p_edge->detection_doration;
                }

                // Full dominance check on all objectives
                if (is_dominated(next, closed_vector[next_id])) {
                    continue;
                }
            }

            // If not dominated create node and push to queue
            open.insert(next);
        }
    }

    this->end_logging(solutions);
}
