#include <memory>
#include <vector>

#include <iostream>

#include "ApexSearch.h"


ApexSearch::ApexSearch(const AdjacencyMatrix &adj_matrix, EPS eps, ApexMode mode, const LoggerPtr logger) :
    AbstractSolver(adj_matrix, eps, logger),
    num_of_objectives(adj_matrix.get_num_of_objectives()),
    mode(mode)
{
    expanded.resize(this->adj_matrix.size()+1);
}

// Compute heuristic based on mode
std::vector<double> ApexSearch::compute_heuristic(const Heuristic& heuristic, size_t node_id) {
    if (mode == ApexMode::ORIGINAL_MO) {
        // Original mode: return full heuristic vector
        return heuristic(node_id);
    } else {
        // Aggregation mode: return simplified vector {0, h[last]}
        auto h = heuristic(node_id);
        return std::vector<double>{0, h[adj_matrix.get_num_of_objectives() - 1]};
    }
}

// Create initial node based on mode
NodePtr ApexSearch::create_initial_node(size_t source, const Heuristic& heuristic) {
    if (mode == ApexMode::ORIGINAL_MO) {
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

void ApexSearch::insert(ApexPathPairPtr &ap, APQueue &queue) {
    std::list<ApexPathPairPtr> &relevant_aps = queue.get_open(ap->id);
    for (auto existing_ap = relevant_aps.begin(); existing_ap != relevant_aps.end(); ++existing_ap) {
        if ((*existing_ap)->is_active == false) {
            continue;
        }
        if (ap->update_nodes_by_merge_if_bounded(*existing_ap, this->eps, ms) == true) {
            // pp and existing_pp were merged successfuly into pp
            // std::cout << "merge!" << std::endl;
            if ((ap-> apex!= (*existing_ap)->apex) ||
                (ap-> path_node!= (*existing_ap)->path_node)) {
                // If merged_pp == existing_pp we avoid inserting it to keep the queue as small as possible.
                // existing_pp is deactivated and not removed to avoid searching through the heap
                // (it will be removed on pop and ignored)
                (*existing_ap)->is_active = false;
                queue.insert(ap);
            }
            // both apex and path_node are equal -> ap is dominated
            return;
        }
    }
    queue.insert(ap);
}


void ApexSearch::merge_to_solutions(const ApexPathPairPtr &ap, ApexPathSolutionSet &solutions) {
    for (auto existing_solution = solutions.begin(); existing_solution != solutions.end(); ++existing_solution) {
        if ((*existing_solution)->update_nodes_by_merge_if_bounded(ap, this->eps, ms) == true) {
            return;
        }
    }
    solutions.push_back(ap);
    // std::cout << "update solution checker" << std::endl;
    solution_dom_checker->add_node(ap);
}


bool ApexSearch::is_dominated(ApexPathPairPtr ap){
    if (local_dom_checker->is_dominated(ap)){
        return true;
    }
    return solution_dom_checker->is_dominated(ap);
}


void ApexSearch::operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit) {

    init_search();

    auto start_time = std::clock();

    if (num_of_objectives == 2){
        local_dom_checker = std::make_unique<LocalCheck>(eps, this->adj_matrix.size());
        solution_dom_checker = std::make_unique<SolutionCheck>(eps);
    }else{
        local_dom_checker = std::make_unique<LocalCheckLinear>(eps, this->adj_matrix.size());
        solution_dom_checker = std::make_unique<SolutionCheckLinear>(eps);
    }


    this->start_logging(source, target);

    ApexPathSolutionSet ap_solutions;
    ApexPathPairPtr   ap;
    ApexPathPairPtr   next_ap;

    // Init open heap
    APQueue open(this->adj_matrix.size()+1);

    // Create initial node using mode-aware heuristic
    NodePtr source_node = create_initial_node(source, heuristic);
    ap = std::make_shared<ApexPathPair>(source_node, source_node, heuristic);
    open.insert(ap);

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            for (auto solution = ap_solutions.begin(); solution != ap_solutions.end(); ++solution) {
                solutions.push_back((*solution)->path_node);

            }

            this->end_logging(solutions, false);
            return;
        }
        // Pop min from queue and process
        ap = open.pop();
        num_generation +=1;

        // Optimization: PathPairs are being deactivated instead of being removed so we skip them.
        if (ap->is_active == false) {
            continue;
        }

        // Dominance check
        if (is_dominated(ap)){
            continue;
        }

        local_dom_checker->add_node(ap);

        num_expansion += 1;

        expanded[ap->id].push_back(ap);

        if (ap->id == target) {
            this->merge_to_solutions(ap, ap_solutions);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[ap->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            // Prepare extension of path pair

            next_ap = std::make_shared<ApexPathPair>(ap, *p_edge);

            // Track edge detection time using canonical edge representation
            // Use canonical edge representation (min, max) to ensure each physical edge
            // is counted exactly once, regardless of which direction we traverse it first.
            // For example, edge 5→7 and edge 7→5 both map to canonical pair (5,7).
            auto canonical_edge = std::minmax(p_edge->source, p_edge->target);
            if (counted_edges.insert(canonical_edge).second) {
                // insert() returns pair<iterator, bool> where .second is true if element was newly inserted
                total_edge_detection_time = total_edge_detection_time + p_edge->detection_doration;
            }

            // Dominance check
            if (is_dominated(next_ap)){
                continue;
            }

            // If not dominated extend path pair and push to queue
            // Creation is deferred after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            this->insert(next_ap, open);
        }
    }

    // Pair solutions is used only for logging, as we need both the solutions for testing reasons
    for (auto solution = ap_solutions.begin(); solution != ap_solutions.end(); ++solution) {
        solutions.push_back((*solution)->path_node);

    }

    this->end_logging(solutions);
}

std::string ApexSearch::get_solver_name() {
    std::string alg_variant;
    if (ms == MergeStrategy::SMALLER_G2){
        alg_variant ="-s2";
    } else if ( ms == MergeStrategy::SMALLER_G2_FIRST){
        alg_variant ="-s2f";
    } else if (ms == MergeStrategy::RANDOM){
        alg_variant ="-r";
    } else if (ms == MergeStrategy::MORE_SLACK){
        alg_variant ="-ms";
    } else if (ms == MergeStrategy::REVERSE_LEX){
        alg_variant ="-rl";
    }

    std::string mode_suffix;
    if (mode == ApexMode::ORIGINAL_MO) {
        mode_suffix = "-OriginalMO";
    } else {
        mode_suffix = "-MOWithAggregation";
    }

    return "Apex" + alg_variant + mode_suffix;
}

void ApexSearch::init_search(){
    AbstractSolver::init_search();
    expanded.clear();
    expanded.resize(this->adj_matrix.size()+1);
    // Clear edge tracking from any previous search
    counted_edges.clear();
}
