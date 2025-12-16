#include "Utils/Definitions.h"
#include <random>

// Dominance check for ORIGINAL_MO mode - starts from g[1]
bool is_dominated_dr_original(NodePtr apex, NodePtr node){
  for (int i = 1; i < apex->g.size(); i ++ ){
    if (node->g[i] > apex->g[i]){
      return false;
    }
  }
  return true;
}

// Dominance check for MO_WITH_AGGREGATION mode - checks all g-values from g[0]
bool is_dominated_dr_aggregation(NodePtr apex, NodePtr node){
  for (int i = 0; i < apex->g.size(); i ++ ){
    if (node->g[i] > apex->g[i]){
      return false;
    }
  }
  return true;
}

// Wrapper function that detects mode based on h-vector size
bool is_dominated_dr(NodePtr apex, NodePtr node){
  // Detect mode: if h-vector size differs from g-vector size, we're in aggregation mode
  if (apex->h.size() != apex->g.size()) {
    // MO_WITH_AGGREGATION mode
    return is_dominated_dr_aggregation(apex, node);
  } else {
    // ORIGINAL_MO mode
    return is_dominated_dr_original(apex, node);
  }
}

// (dominatee ,dominator)
bool is_dominated_dr(NodePtr apex, NodePtr node, const EPS eps){
  for (int i = 1; i < apex->f.size(); i ++ ){
    if (node->f[i] > (1 + eps[i]) * apex->f[i]){
      return false;
    }
  }
  return true;
}


bool is_bounded(NodePtr apex, NodePtr node,  const EPS eps){
  for (int i = 0; i < apex->f.size(); i ++ ){
    if (node->f[i] > (1 + eps[i]) * apex->f[i]){
      return false;
    }
  }
  return true;
}

double compute_slack(NodePtr apex, NodePtr node,  const EPS eps){
  double min_slack = ( (1 + eps[0]) - (double)node->g[0] / (double) apex->g[0] ) / eps[0];
  for (int i = 1; i < apex->g.size(); i ++ ){

    double slack = ( (1 + eps[i]) - (double)node->g[i] / (double) apex->g[i] ) / eps[i];
    if (slack < min_slack){
      min_slack = slack;
    }
  }
  return min_slack;
}


ApexPathPair::ApexPathPair(const ApexPathPairPtr parent, const Edge&  edge): h(parent->h){
  size_t next_id = edge.target;
  id =next_id;

  std::vector<double> new_apex_g(parent->apex->g);
  std::vector<double> new_g(parent->path_node->g);

  // Max for first n-1 objectives (risks), sum for last objective (length)
  for (int i = 0; i < new_apex_g.size() - 1; i ++){
    new_apex_g[i] = std::max(new_apex_g[i], edge.cost[i]);
    new_g[i] = std::max(new_g[i], edge.cost[i]);
  }
  // Sum for last objective (length)
  new_apex_g[new_apex_g.size() - 1] += edge.cost[new_apex_g.size() - 1];
  new_g[new_g.size() - 1] += edge.cost[new_g.size() - 1];

  // Compute heuristic based on mode (detect from parent's h-vector size)
  auto full_h = h(next_id);
  std::vector<double> new_h;
  if (parent->apex->h.size() != parent->apex->g.size()) {
    // MO_WITH_AGGREGATION mode: use simplified h-vector {0, h[last]}
    new_h = std::vector<double>{0, full_h[full_h.size() - 1]};
  } else {
    // ORIGINAL_MO mode: use full h-vector
    new_h = full_h;
  }

  this->apex = std::make_shared<Node>(next_id, new_apex_g, new_h);
  this->path_node = std::make_shared<Node>(next_id, new_g, new_h);
}


bool ApexPathPair::update_nodes_by_merge_if_bounded(const ApexPathPairPtr &other, const std::vector<double> eps, MergeStrategy s){
  // Returns true on sucessful merge and false if it failure
  if (this->id != other->id) {
    return false;
  }

  NodePtr new_apex = std::make_shared<Node>(this->apex->id, this->apex->g, this->apex->h);
  NodePtr new_path_node = nullptr;

  // Merge apex
  if (new_apex->h.size() != new_apex->g.size()) {
    // MO_WITH_AGGREGATION mode: merge g-values, then recompute f
    for (int i = 0; i < other->apex->g.size(); i ++){
      if (other->apex->g[i] < new_apex->g[i]){
        new_apex->g[i] = other->apex->g[i];
      }
    }
    // Recompute f-values from merged g-values
    auto risk_vector_size = new_apex->g.size()-1;
    double safety_value = 1;
    for(size_t i = 0; i < risk_vector_size; i++){
      safety_value = safety_value * (1-new_apex->g[i]);
    }
    double risk_value = 1-safety_value;
    new_apex->f[0] = risk_value;
    new_apex->f[1] = new_apex->g[risk_vector_size] + new_apex->h[1];
  } else {
    // ORIGINAL_MO mode: keep original logic
    for (int i = 0; i < other->apex->g.size(); i ++){
      if (other->apex->g[i] < new_apex->g[i]){
        new_apex->g[i] = other->apex->g[i];
        new_apex->f[i] = other->apex->f[i];
      }
    }
  }

  // Choose a path node using RANDOM strategy
  if (is_bounded(new_apex, this->path_node, eps)){
    new_path_node = this->path_node;
  }
  if (is_bounded(new_apex, other->path_node, eps)){
    if (new_path_node == nullptr){
      new_path_node = other->path_node;
    }else{
      if (rand() % 2 == 1){
        new_path_node = other->path_node;
      }
    }
  }
  if (new_path_node == nullptr){
    return false;
  }

  this->apex = new_apex;
  this->path_node = new_path_node;
  return true;
}

bool ApexPathPair::update_apex_by_merge_if_bounded(const NodePtr &other_apex, const std::vector<double> eps){
  NodePtr new_apex = std::make_shared<Node>(this->apex->id, this->apex->g, this->apex->h);
  bool update_flag = false;

  // Important: Here we merge according to the aggregation functions!

  // Merge apex
  if (new_apex->h.size() != new_apex->g.size()) {
    // MO_WITH_AGGREGATION mode: Compare aggregated safety and length separately
    size_t vec_size = other_apex->g.size();
    size_t last_index = vec_size - 1;

    // Safety value: take minimum
    if (other_apex->f[0] < new_apex->f[0]) {
      // Copy all risk g-values and f-values from other
      for (size_t i = 0; i < last_index; ++i) {
        new_apex->g[i] = other_apex->g[i];
      }
      new_apex->f[0] = other_apex->f[0];  // Copy aggregated risk
      update_flag = true;
    }

    // Length value: take minimum 
    if (other_apex->f[1] < new_apex->f[1]) {
      new_apex->g[1] = other_apex->g[1];
      new_apex->f[1] = other_apex->f[1];


      update_flag = true;
    }
    // Check if path_node is still bounded by new apex length
    for (size_t i = 0; i < path_node->f.size(); ++i) {
      if (path_node->f[i] > (1 + eps[i]) * new_apex->f[i]) {
        return false;
      }
    }


  } else {
    // ORIGINAL_MO mode: keep original logic
    for (int i = 0; i < other_apex->g.size(); i ++){
      if (other_apex->f[i] < new_apex->f[i]){
        new_apex->g[i] = other_apex->f[i];
        new_apex->f[i] = other_apex->f[i];
        if ( path_node->f[i] > (1 + eps[i]) * new_apex->f[i] ){
          return false;
        }

        update_flag = true;
      }
    }
  }

  if (update_flag){
    apex = new_apex;
  }
  return true;
}



bool ApexPathPair::more_than_full_cost::operator()(const ApexPathPairPtr &a, const ApexPathPairPtr &b) const {
  return Node::more_than_full_cost()(a->apex, b->apex);
}


std::ostream& operator<<(std::ostream &stream, const ApexPathPair &ap) {
  // Printed in JSON format
  stream << "{" << *(ap.apex) << ", " << *(ap.path_node) << "}";
  return stream;
}
