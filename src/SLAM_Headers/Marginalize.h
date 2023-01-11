#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

using namespace gtsam;
using namespace std;

void AddFactors(const gtsam::GaussianFactorGraph& newFactors, gtsam::NonlinearFactorGraph& graph) {
  for(const auto& f: newFactors){
    gtsam::LinearContainerFactor linearContainer(f);
    gtsam::NonlinearFactorGraph fg = linearContainer.ConvertLinearGraph(newFactors);
    for(const auto& nlfactor: fg){
    graph.add(nlfactor);
    }
    break;
  }
}

gtsam::KeyVector AllButOne(gtsam::Key key, const gtsam::GaussianFactorGraph& graph){
  auto it = graph.begin();
  std::set<gtsam::Key> keys;
  for (auto factor: graph){
    for(auto k: factor->keys()){
      if(k!=key){
         keys.insert(k);
      }
    }
  }
  gtsam::KeyVector ret;
  for(gtsam::Key key: keys){
    ret.push_back(key);
  }
  return ret;
}

bool KeyInKeyVector(const KeyVector& keys, Key k){
  //Returns if a key is in the key vector
  for (auto key: keys){
    if (k == key){
      return true;
    }
  }
  return false;
}

void EraseKeys(const gtsam::KeyVector& keys, gtsam::NonlinearFactorGraph& graph) {
  //Modifies graph by erasing keys and any factors connected to the keys
  auto it = graph.begin();
  int size = graph.size();
  for (auto key: keys){
    int num = 0;
    for (auto factor: graph){
      bool a = KeyInKeyVector(factor->keys(), key);
      if(a){
        graph.erase(it+num);
      }
      //TODO: Why is this necessary?
      if (num == graph.size()-1){
        break;
      }
      num++;
    }
  }
}

void Marginalize(const gtsam::Key &key, gtsam::NonlinearFactorGraph& graph, const gtsam::Values &linearization_point) {

  // Marginalizes specified keys from a nonlinear factor graph by computing the subgraph containing the marginalized keys
  // and their attached factors, linearizing this graph, applying marginalization upon the linearized graph, and adding the 
  // resulting factors back into the original graph.

  // Determine the factors touching `key` and put them in a separate graph
  std::set<size_t> removedFactorSlots;
  const gtsam::VariableIndex variableIndex(graph);
  const auto& slots = variableIndex[key];
  removedFactorSlots.insert(slots.begin(), slots.end());

  gtsam::NonlinearFactorGraph removedFactors;
  for(size_t slot: removedFactorSlots) {
    if (graph.at(slot)) {
      removedFactors.add(graph.at(slot));
    }
  }

  // Linearize the graph of factors touching the key
  gtsam::KeyVector k = {key};
  auto g_ptr = removedFactors.linearize(linearization_point);
  gtsam::GaussianFactorGraph g = *g_ptr;

   //Marginalize this graph
  gtsam::GaussianFactorGraph m = *(g.marginal(AllButOne(key,g)));

  // //Remove the marginalized keys from the original graph
  EraseKeys(k, graph);

  //Add induced factors to the graph
  AddFactors(m, graph);
}

