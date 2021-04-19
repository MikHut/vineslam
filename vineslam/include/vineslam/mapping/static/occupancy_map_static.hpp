#pragma once

#include <map>
#include <vector>

#include <vineslam/feature/semantic.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/feature/visual.hpp>

namespace vineslam
{
struct CellRoutines
{
// Inserts a landmark with a given id
static void insert(const int& id, const SemanticFeature& l_landmark, std::map<int, SemanticFeature>* landmarks)
{
  (*landmarks)[id] = l_landmark;
}

// Inserts a image feature in the features array
static void insert(const ImageFeature& l_feature, std::vector<ImageFeature>* surf_features)
{
  surf_features->push_back(l_feature);
}

// Inserts a corner feature in the features array
static void insert(const Corner& l_feature, std::vector<Corner>* corner_features)
{
  corner_features->push_back(l_feature);
}

// Inserts a planar feature in the features array
static void insert(const Planar& l_feature, std::vector<Planar>* planar_features)
{
  planar_features->push_back(l_feature);
}
};

}  // namespace vineslam