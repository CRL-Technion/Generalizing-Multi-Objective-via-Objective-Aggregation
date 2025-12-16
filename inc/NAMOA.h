#pragma once

#include <vector>
#include "Utils/Definitions.h"
#include "Utils/Logger.h"
#include "AbstractSolver.h"

// NAMOAdr algorithm modes
enum class NAMOAdrMode {
    ORIGINAL_MO,        // Original multi-objective: all objectives treated independently
    MO_WITH_AGGREGATION // Multi-objective with aggregation: risk objectives aggregated into safety
};

class NAMOAdr: public AbstractSolver {
private:
    NAMOAdrMode mode;

    // Set to track which edges have already been counted for detection time.
    // Uses canonical edge representation (min_node, max_node) to ensure each
    // physical edge is only counted once, regardless of traversal direction.
    std::unordered_set<std::pair<size_t, size_t>, PairHash> counted_edges;

public:
    double total_edge_detection_time = 0;

    NAMOAdr(const AdjacencyMatrix &adj_matrix, EPS eps, NAMOAdrMode mode, const LoggerPtr logger=nullptr);

    virtual std::string get_solver_name() override;

    void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override;

private:
    // Helper methods for mode-specific behavior
    std::vector<double> compute_heuristic(const Heuristic& heuristic, size_t node_id);
    NodePtr create_initial_node(size_t source, const Heuristic& heuristic);
};


