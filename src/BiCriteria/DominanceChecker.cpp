#include "DominanceChecker.h"


bool SolutionCheck::is_dominated(ApexPathPairPtr node){
    if (last_solution == nullptr){
        return false;
    }
    if (is_bounded(node->apex, last_solution->path_node, eps)){
        assert(last_solution->update_apex_by_merge_if_bounded(node->apex, eps));
        // std::cout << "solution dom" << std::endl;
        return true;
    }
    return false;
}


bool LocalCheck::is_dominated(ApexPathPairPtr node){
    return (node->apex->g[1] >= min_g2[node->id]);
}

void LocalCheck::add_node(ApexPathPairPtr ap){
    auto id = ap->id;
    assert(min_g2[ap->id] > ap->apex->g[1]);
    min_g2[ap->id] = ap->apex->g[1];
}

bool LocalCheckLinear::is_dominated(ApexPathPairPtr node){
    for (auto ap:min_g2[node->id]){
        if (is_dominated_dr(node->apex, ap->apex)){
            assert(node->apex->f[0] >= ap->apex->f[0]);
            return true;
        }
    }
    return false;
}

void LocalCheckLinear::add_node(ApexPathPairPtr ap){
    auto id = ap->id;
    for (auto it = min_g2[id].begin(); it != min_g2[id].end(); ){
        // TODO remove it for performance
        assert(! is_dominated_dr(ap->apex, (*it)->apex  ));
        if (is_dominated_dr((*it)->apex, ap->apex)){
            it = min_g2[id].erase(it);
        } else {
            it ++;
        }
    }

    min_g2[ap->id].push_front(ap);
}

bool SolutionCheckLinear::is_dominated(ApexPathPairPtr node){
    for (auto ap: solutions){
        // if (is_bounded(node->apex, ap->path_node, eps)){
        if (ap->update_apex_by_merge_if_bounded(node->apex, eps)){
            // assert(ap->update_apex_by_merge_if_bounded(node->apex, eps));
            return true;
        }
    }
    return false;
}

void SolutionCheckLinear::add_node(ApexPathPairPtr ap){
    for (auto it = solutions.begin(); it != solutions.end(); ){
        if (is_dominated_dr((*it)->path_node, ap->path_node)){
            it = solutions.erase(it);
        } else {
            it ++;
        }
    }
    solutions.push_front(ap);
}
