#include <iostream>
#include <memory>
#include <time.h>
#include <fstream>

#include "ShortestPathHeuristic.h"
#include "Utils/Definitions.h"
#include "Utils/IOUtils.h"
#include "Utils/Logger.h"
#include "ApexSearch.h"
#include "NAMOA.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

using namespace std;

const std::string resource_path = "resources/";
const std::string output_path = "output/";
const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::SMALLER_G2;
std::string alg_variant = "";

#include <vector>
#include <memory>
#include <utility>
#include <limits>
#include <algorithm>
#include <iostream>

static inline std::pair<double,double> costs_from_g(const std::vector<double>& g) {
    const size_t n = g.size();
    if (n < 2) {
        // Not enough dims: return worst-possible for our mixed objective
        return { -std::numeric_limits<double>::infinity(),  // worst for max
                  std::numeric_limits<double>::infinity()}; // worst for min
    }
    double c1 = 1.0;                     // maximize
    for (size_t i = 0; i + 1 < n; ++i) { // product over first n-1
        c1 *= (1.0 - g[i]);
    }
    double c2 = g[n - 1];                // minimize
    return {c1, c2};
}

// Dominance for mixed objectives: c1 (max), c2 (min) — STRICT, no eps
static inline bool dominates_mixed_max_min(const std::pair<double,double>& a,
                                           const std::pair<double,double>& b) {
    bool no_worse       = (a.first >= b.first) && (a.second <= b.second);
    bool strictly_better= (a.first >  b.first) || (a.second <  b.second);
    return no_worse && strictly_better;
}

// Extract Pareto frontier (non-dominated set) for c1=maximize, c2=minimize
inline std::vector<NodePtr> pareto_frontier_mixed(const SolutionSet& solutions) {
    const size_t m = solutions.size();
    std::vector<std::pair<double,double>> costs(m);
    for (size_t i = 0; i < m; ++i) costs[i] = costs_from_g(solutions[i]->g);

    std::vector<NodePtr> frontier;
    frontier.reserve(m);

    for (size_t i = 0; i < m; ++i) {
        bool dominated = false;
        for (size_t j = 0; j < m && !dominated; ++j) {
            if (j == i) continue;
            if (dominates_mixed_max_min(costs[j], costs[i])) {
                dominated = true;
            }
        }
        if (!dominated) frontier.push_back(solutions[i]);
    }

    // Optional: nice ordering — c1 DESC (higher first), then c2 ASC (lower first)
    std::sort(frontier.begin(), frontier.end(),
        [&](const NodePtr& a, const NodePtr& b){
            auto ca = costs_from_g(a->g);
            auto cb = costs_from_g(b->g);
            if (ca.first != cb.first) return ca.first > cb.first; // higher c1 first
            return ca.second < cb.second;                         // lower c2 first
        });

    return frontier;
}

// Printing helpers
inline void print_solution_costs(const NodePtr& n) {
    auto [c1, c2] = costs_from_g(n->g);
    std::cout << "Node id=" << n->id
              << "  c1(prod(1-g[0..n-2]))=" << c1
              << "  c2(g[n-1])=" << c2
              << "\n";
}

inline void print_frontier(const std::vector<NodePtr>& frontier) {
    std::cout << "Pareto frontier (" << frontier.size() << " solutions):\n";
    for (const auto& n : frontier) {
        print_solution_costs(n);
    }
}

// Simple example to demonstrate the usage of the algorithm

void single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, unsigned int time_limit, std::string namoa_mode = "original", std::string apex_mode = "original") {
    // Compute heuristic
    std::cout << "Source: " << source << ", Target: " << target << std::endl;
    std::cout << "Time limit: " << time_limit << " seconds" << std::endl;
    const TimePoint start_a = Clock::now();
    auto runtime = std::clock();  // Start timing to include heuristic computation

    std::cout << "Starting heuristic computation..." << std::endl;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
    // sp_heuristic.set_all_to_zero();
    std::cout << "Heuristic computation completed.\n" << std::endl;

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, _1);

    SolutionSet solutions;
    int num_exp, num_gen;

    std::unique_ptr<AbstractSolver> solver;
    std::string namoa_mode_used = ""; // Track which NAMOA mode was used

    if (algorithm == "NAMOAdr"){
        // Runtime mode selection based on command-line parameter
        namoa_mode_used = namoa_mode;
        EPS eps_vec;
        NAMOAdrMode mode_enum;

        if (namoa_mode_used == "original") {
            // Original multi-objective mode
            mode_enum = NAMOAdrMode::ORIGINAL_MO;
            eps_vec = EPS(graph.get_num_of_objectives() - 1, 0);
            eps_vec.push_back(eps);
            std::cout << "Running NAMOAdr in Original Multi-Objective mode" << std::endl;
        } else if (namoa_mode_used == "aggregation") {
            // Multi-objective with aggregation mode
            mode_enum = NAMOAdrMode::MO_WITH_AGGREGATION;
            eps_vec = EPS{0, eps};
            std::cout << "Running NAMOAdr in Multi-Objective with Aggregation mode" << std::endl;
        } else {
            std::cerr << "Unknown NAMOAdr mode: " << namoa_mode_used << ". Use 'original' or 'aggregation'." << std::endl;
            exit(-1);
        }

        solver = std::make_unique<NAMOAdr>(graph, eps_vec, mode_enum, logger);

        std::cout << "Epsilon vector: ";
        for (auto a : eps_vec) {
            std::cout << a << " ";
        }
        std::cout << std::endl;
    }else if (algorithm == "Apex"){
        // Runtime mode selection based on command-line parameter
        std::string apex_mode_used = apex_mode;
        EPS eps_vec;
        ApexMode mode_enum;

        if (apex_mode_used == "original") {
            // Original multi-objective mode
            mode_enum = ApexMode::ORIGINAL_MO;
            eps_vec = EPS(graph.get_num_of_objectives() - 1, 0);
            eps_vec.push_back(eps);
            std::cout << "Running Apex in Original Multi-Objective mode" << std::endl;
        } else if (apex_mode_used == "aggregation") {
            // Multi-objective with aggregation mode
            mode_enum = ApexMode::MO_WITH_AGGREGATION;
            eps_vec = EPS{0, eps};
            std::cout << "Running Apex in Multi-Objective with Aggregation mode" << std::endl;
        } else {
            std::cerr << "Unknown Apex mode: " << apex_mode_used << ". Use 'original' or 'aggregation'." << std::endl;
            exit(-1);
        }

        solver = std::make_unique<ApexSearch>(graph, eps_vec, mode_enum, logger);
        ((ApexSearch*)solver.get())->set_merge_strategy(ms);

        std::cout << "Epsilon vector: ";
        for (auto a : eps_vec) {
            std::cout << a << " ";
        }
        std::cout << std::endl;
    }else{
        std::cerr << "unknown solver name" << std::endl;
        exit(-1);
    }
    (*solver)(source, target, heuristic, solutions, time_limit);
    runtime = std::clock() - runtime;  // End timing - includes heuristic computation

    auto doration_apex = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start_a).count();
    std::cout << "Runtime before adding edge detection time: " << doration_apex << " ms" << std::endl;

    // Add edge detection time for both NAMOAdr and Apex
    if (algorithm == "NAMOAdr") {
        doration_apex = doration_apex + static_cast<NAMOAdr*>(solver.get())->total_edge_detection_time;
    } else if (algorithm == "Apex") {
        doration_apex = doration_apex + static_cast<ApexSearch*>(solver.get())->total_edge_detection_time;
    }

    std::cout << "Runtime after adding edge detection time: " << doration_apex << " ms" << std::endl;
    auto frontier = pareto_frontier_mixed(solutions);

    std::cout << "Node expansions: " << solver->get_num_expansion() << std::endl;
    num_exp = solver->get_num_expansion();
    num_gen = solver->get_num_generation();

    // Print solution header based on mode
    if (algorithm == "NAMOAdr") {
        if (namoa_mode_used == "original") {
            std::cout << "\nSolutions (full_cost = per-objective costs):" << std::endl;
        } else if (namoa_mode_used == "aggregation") {
            std::cout << "\nSolutions (full_cost = aggregated function costs):" << std::endl;
        }
    } else {
        std::cout << "\nSolutions:" << std::endl;
    }

    for (auto sol: solutions){
        std::cout << *sol << std::endl;
    }
    std::cout << "Number of solutions: " << solutions.size() << std::endl;

    output << algorithm << "-" << alg_variant << " (" << eps << ")" << "\t"
           << source << "\t" << target << "\t"
           << num_gen << "\t"
           << num_exp << "\t"
           << solutions.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;

    std::cout << "----- End of single example -----" << std::endl;
}

void single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, unsigned int time_limit, std::string namoa_mode = "original", std::string apex_mode = "original") {
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);

    single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, namoa_mode, apex_mode);
 }

void run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, unsigned int time_limit, std::string namoa_mode = "original", std::string apex_mode = "original") {
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);


    std::vector<std::pair<size_t, size_t>> queries;
    if (load_queries(query_file, queries) == false) {
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }

    // Build graphs
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);

    size_t query_count = 0;
    for (auto iter = queries.begin(); iter != queries.end(); ++iter) {

        query_count++;
        std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
        size_t source = iter->first;
        size_t target = iter->second;

        single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, namoa_mode, apex_mode);
    }

}

int main(int argc, char** argv){
#ifdef NDEBUG
    std::cout << "Release build\n";
#else
    std::cout << "Debug build\n";
#endif
    namespace po = boost::program_options;

    std::vector<string> objective_files;
    std::string edge_times_files;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("start,s", po::value<int>()->default_value(-1), "start location")
        ("goal,g", po::value<int>()->default_value(-1), "goal location")
        ("query,q", po::value<std::string>()->default_value(""), "number of agents")
        ("map,m",po::value< std::vector<string> >(&objective_files)->multitoken(), "files for edge weight")
        ("eps,e", po::value<double>()->default_value(0), "approximation factor")
        ("merge", po::value<std::string>()->default_value(""), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
        ("algorithm,a", po::value<std::string>()->default_value("Apex"), "solvers (Apex or NAMOAdr)")
        ("namoa-mode", po::value<std::string>()->default_value("original"), "NAMOAdr mode: original or aggregation")
        ("apex-mode", po::value<std::string>()->default_value("original"), "Apex mode: original or aggregation")
        ("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
        ("output,o", po::value<std::string>()->required(), "Name of the output file")
        ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
        ("e_time,e",po::value<std::string>(&edge_times_files)->default_value("-"), "files for edge durations time")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    srand((int)time(0));

    if (vm["query"].as<std::string>() != ""){
        if (vm["start"].as<int>() != -1 || vm["goal"].as<int>() != -1){
            std::cerr << "query file and start/goal cannot be given at the same time !" << std::endl;
            return -1;
        }
    }
    
    LoggerPtr logger = nullptr;

    if (vm["logging_file"].as<std::string>() != ""){
        logger = new Logger(vm["logging_file"].as<std::string>());
    }

    // Load files
    size_t graph_size;
    std::vector<Edge> edges;

    for (auto file:objective_files){
        std::cout << file << std::endl;
    }

    if (load_gr_files(objective_files, edges, graph_size) == false) {
        std::cout << "Failed to load gr files" << std::endl;
        return -1;
    }
    if(edge_times_files != "-"){
        load_edge_doration_file(edge_times_files, edges);
    }

    std::cout << "Graph Size: " << graph_size << std::endl;

    // Build graphs
    MergeStrategy ms = DEFAULT_MERGE_STRATEGY;
    alg_variant = vm["merge"].as<std::string>();

    if (vm["merge"].as<std::string>() != "" && vm["algorithm"].as<std::string>()!= "Apex"){
        alg_variant = "";
        std::cout << "WARNING: merge strategy with non-apex search" << std::endl;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2"){
        ms = MergeStrategy::SMALLER_G2;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2_FIRST"){
        ms = MergeStrategy::SMALLER_G2_FIRST;
    }else if(vm["merge"].as<std::string>() == "RANDOM"){
        ms = MergeStrategy::RANDOM;
    }else if(vm["merge"].as<std::string>() == "MORE_SLACK"){
        ms = MergeStrategy::MORE_SLACK;
    }else if(vm["merge"].as<std::string>() == "REVERSE_LEX"){
        ms = MergeStrategy::REVERSE_LEX;
    }else{
        std::cerr << "unknown merge strategy" << std::endl;
    }


    if (vm["query"].as<std::string>() != ""){
        run_query(graph_size, edges, vm["query"].as<std::string>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), vm["namoa-mode"].as<std::string>(), vm["apex-mode"].as<std::string>());
    } else{
        single_run_map(graph_size, edges, vm["start"].as<int>(), vm["goal"].as<int>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>(), vm["namoa-mode"].as<std::string>(), vm["apex-mode"].as<std::string>());
    }

    delete(logger);

    return 0;
}
