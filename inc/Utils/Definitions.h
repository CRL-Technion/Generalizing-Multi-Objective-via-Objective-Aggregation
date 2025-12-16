#ifndef UTILS_DEFINITIONS_H
#define UTILS_DEFINITIONS_H

#include <map>
#include <vector>
#include <array>
#include <list>
#include <iostream>
#include <limits>
#include <functional>
#include <memory>
#include <climits>
#include <unordered_set>
#include "boost/heap/pairing_heap.hpp"
#include "boost/heap/priority_queue.hpp"


#ifndef DEBUG
#define DEBUG 1
#endif

// NAMOA Algorithm Modes:
// The NAMOAdr algorithm supports two operational modes that can be selected at runtime
// via the --namoa-mode command-line parameter:
//
// MODE 1 - Original Multi-Objective (--namoa-mode=original):
//   Standard multi-objective A* treating all objectives independently.
//   Each objective (risk values + cost) is handled separately.
//
// MODE 2 - Multi-Objective with Aggregation (--namoa-mode=aggregation):
//   Aggregates multiple risk objectives into a single safety objective.
//   The aggregated safety and cost form a 2-objective problem.
//
// NOTE: Both modes are compiled into the binary and can be selected at runtime.
// No recompilation needed to switch modes!

const size_t MAX_COST = std::numeric_limits<size_t>::max();


template<typename T>
using Pair      = std::array<T, 2>;

template<typename T>
std::ostream& operator<<(std::ostream &stream, const Pair<T> pair) {
    stream << "[" << pair[0] << ", " << pair[1] << "]";
    return stream;
}


using Heuristic = std::function<std::vector<double>(size_t)>;

// Hash function for std::pair<size_t, size_t> to use in unordered_set
// This allows us to efficiently track which edges have been counted for detection time
struct PairHash {
    std::size_t operator()(const std::pair<size_t, size_t>& p) const {
        // Combine two hash values using a simple but effective method
        // The magic number 0x9e3779b9 helps distribute hash values uniformly
        std::size_t h1 = std::hash<size_t>{}(p.first);
        std::size_t h2 = std::hash<size_t>{}(p.second);
        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
};

// Structs and classes
struct Edge {
    size_t          source;
    size_t          target;

    std::vector<double>    cost;
    double detection_doration = 0;

    Edge(size_t source, size_t target, std::vector<double> cost) : source(source), target(target), cost(cost) {}
    Edge inverse() {
        return Edge(this->target, this->source, this->cost);
    }
    void set_doration(double doration){
        detection_doration = doration;
    };
};
std::ostream& operator<<(std::ostream &stream, const Edge &edge);


// Graph representation as adjacency matrix
class AdjacencyMatrix {
private:
    std::vector<std::vector<Edge>> matrix;
    size_t                         graph_size;
    size_t num_of_objectives = 0;

public:
    AdjacencyMatrix() = default;
    AdjacencyMatrix(size_t graph_size, std::vector<Edge> &edges, bool inverse=false);
    void add(Edge edge);
    size_t size(void) const;
    size_t get_num_of_objectives() const;
    const std::vector<Edge>& operator[](size_t vertex_id) const;

    friend std::ostream& operator<<(std::ostream &stream, const AdjacencyMatrix &adj_matrix);
};


struct Node;
struct PathPair;
struct ApexPathPair;
using NodePtr       = std::shared_ptr<Node>;
using PathPairPtr   = std::shared_ptr<PathPair>;
using ApexPathPairPtr   = std::shared_ptr<ApexPathPair>;
using SolutionSet   = std::vector<NodePtr>;
using PPSolutionSet = std::vector<PathPairPtr>;
using ApexPathSolutionSet = std::vector<ApexPathPairPtr>;

using EPS = std::vector<double>;

struct Node {
    size_t          id;
    std::vector<double>    g;
    std::vector<double>    h;
    std::vector<double>    f;
    NodePtr         parent;

    Node(size_t id, std::vector<double> g, std::vector<double> h, NodePtr parent=nullptr)
        : id(id), g(g), h(h), f(h.size()), parent(parent) {
        // Detect mode based on h-vector size:
        // - MOWithAggregationMode: h size differs from g size
        // - OriginalMOMode: h has same dimensions as g
        if (h.size() != g.size()) {
            // MOWithAggregationMode: compute aggregated safety from g-vector
            auto risk_vector_size = g.size()-1;
            double safety_value = 1;
            for(size_t i = 0; i < risk_vector_size; i++){
                safety_value = safety_value * (1-g[i]);
            }
            double risk_value = 1-safety_value;
            f[0] = risk_value;
            f[1] = g[risk_vector_size] + h[1];
        } else {
            // OriginalMOMode: standard f = g + h
            for (size_t i = 0; i < h.size(); i++){
                f[i] = g[i] + h[i];
            }
        }
    };

    struct more_than_specific_heurisitic_cost {
        size_t cost_idx;

        more_than_specific_heurisitic_cost(size_t cost_idx) : cost_idx(cost_idx) {};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };

    struct more_than_combined_heurisitic {
        double factor;

        more_than_combined_heurisitic(double factor) : factor(factor) {};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };


    struct more_than_full_cost {
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };

    enum LEX_ORDER {LEX0, LEX1};
    struct more_than_lex{
        Node::LEX_ORDER order;
        more_than_lex(Node::LEX_ORDER order) : order(order) {};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };


    struct compare_lex1
    {
        bool operator()(const NodePtr n1, const NodePtr n2) const
        {
            if (n1->f[0] != n2->f[0]){
                return n1->f[0] > n2->f[0];
            }
            return n1->f[1] > n2->f[1];
        }
    };
    friend std::ostream& operator<<(std::ostream &stream, const Node &node);
};


struct PathPair {
    size_t      id;
    NodePtr     top_left;
    NodePtr     bottom_right;
    NodePtr     parent;
    bool        is_active=true;

    PathPair(const NodePtr &top_left, const NodePtr &bottom_right)
        : id(top_left->id), top_left(top_left), bottom_right(bottom_right), parent(top_left->parent) {};

    bool update_nodes_by_merge_if_bounded(const PathPairPtr &other, const Pair<double> eps);
    bool update_nodes_by_merge_if_bounded_keep_track(const PathPairPtr &other, const Pair<double> eps, std::list<NodePtr>& pruned_list);
    bool update_nodes_by_merge_if_bounded2(const PathPairPtr &other, const Pair<double> eps);

    bool if_merge_bounded(const PathPairPtr &other, const Pair<double> eps)  const;


    struct more_than_full_cost {
 bool operator()(const PathPairPtr &a, const PathPairPtr &b) const;
    };

    friend std::ostream& operator<<(std::ostream &stream, const PathPair &pp);
};

enum MergeStrategy {SMALLER_G2, RANDOM, MORE_SLACK, SMALLER_G2_FIRST, REVERSE_LEX};

struct ApexPathPair {
    size_t      id; // state of the node
    NodePtr     apex;
    NodePtr     path_node;
    NodePtr     parent;
    bool        is_active=true;

    Heuristic& h;

    ApexPathPair(const NodePtr &apex, const NodePtr &path_node, Heuristic& h)
        : id(apex->id), apex(apex), path_node(path_node), parent(path_node->parent), h(h){};

    ApexPathPair(const ApexPathPairPtr parent, const Edge& egde);


    bool update_nodes_by_merge_if_bounded(const ApexPathPairPtr &other, const EPS eps, MergeStrategy s=MergeStrategy::SMALLER_G2);
    bool update_apex_by_merge_if_bounded(const NodePtr &other_apex, const EPS eps);

    // bool if_merge_bounded(const ApexPathPairPtr &other, const EP S eps)  const;


    struct more_than_full_cost {
        bool operator()(const ApexPathPairPtr &a, const ApexPathPairPtr &b) const;
    };

    friend std::ostream& operator<<(std::ostream &stream, const ApexPathPair &pp);
};

bool is_bounded(NodePtr apex, NodePtr node,  const EPS eps);
bool is_bounded(NodePtr apex, NodePtr node);
bool is_dominated_dr(NodePtr apex, NodePtr node);
bool is_dominated_dr(NodePtr apex, NodePtr node, const EPS eps);



class Interval{
public:
    double eps = 0;
    NodePtr top_left;
    NodePtr bottom_right;
    std::shared_ptr<std::list<NodePtr>> to_expand;

    Interval(){};
    Interval(const NodePtr top_left, const NodePtr bottom_right, std::shared_ptr<std::list<NodePtr>> to_expand);
};

std::ostream& operator<<(std::ostream& os, const Interval& interval);


using IntervalList   = std::vector<Interval>;

typedef boost::heap::priority_queue<NodePtr , boost::heap::compare<Node::compare_lex1> > heap_open_t;


#endif //UTILS_DEFINITIONS_H
