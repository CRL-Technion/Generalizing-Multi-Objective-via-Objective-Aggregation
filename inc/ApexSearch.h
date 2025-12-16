#pragma once
#include "Utils/Definitions.h"
#include "Utils/Logger.h"
#include "Utils/MapQueue.h"
#include"DominanceChecker.h"
#include "AbstractSolver.h"

// Apex algorithm modes
enum class ApexMode {
    ORIGINAL_MO,        // Original multi-objective: all objectives treated independently
    MO_WITH_AGGREGATION // Multi-objective with aggregation: risk objectives aggregated into safety
};

class ApexSearch: public AbstractSolver {
protected:
    size_t num_of_objectives;
    MergeStrategy ms=MergeStrategy::SMALLER_G2;
    ApexMode mode;

    std::unique_ptr<DominanceChecker> local_dom_checker;
    std::unique_ptr<DominanceChecker> solution_dom_checker;

    virtual void insert(ApexPathPairPtr &pp, APQueue &queue);
    bool is_dominated(ApexPathPairPtr ap);
    void merge_to_solutions(const ApexPathPairPtr &pp, ApexPathSolutionSet &solutions);
    std::vector<std::vector<ApexPathPairPtr>> expanded;
    void init_search();

    // Helper methods for mode-specific behavior
    std::vector<double> compute_heuristic(const Heuristic& heuristic, size_t node_id);
    NodePtr create_initial_node(size_t source, const Heuristic& heuristic);

    // Set to track which edges have already been counted for detection time.
    // Uses canonical edge representation (min_node, max_node) to ensure each
    // physical edge is only counted once, regardless of traversal direction.
    std::unordered_set<std::pair<size_t, size_t>, PairHash> counted_edges;

public:

    virtual std::string get_solver_name();

    double total_edge_detection_time = 0;


    void set_merge_strategy(MergeStrategy new_ms){ms = new_ms;}
    ApexSearch(const AdjacencyMatrix &adj_matrix, EPS eps, ApexMode mode, const LoggerPtr logger=nullptr);
    virtual void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override;
};

