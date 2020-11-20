#pragma once

#include "feature.hpp"

namespace vineslam
{

// ---------------------------------------------------------------------------------
// ----- Semantic high-level feature
// ---------------------------------------------------------------------------------

// Structure to represent the semantic information about
// each feature:
// - type of feature
// - description of the feature
// - dynamic or static
struct SemanticInfo {
  SemanticInfo() = default;
  SemanticInfo(const std::string& type,
               const std::string& description,
               const int&         character)
  {
    (*this).type        = type;
    (*this).description = description;
    (*this).character   = character;
  }

  // Initialize semantic information of feature to give
  // to the mapper class
  explicit SemanticInfo(const int& label)
  {
    std::string m_type;
    std::string m_desc;
    int         m_ch;

    switch (label) {
      case 0:
        m_type = "Trunk";
        m_desc = "Vine trunk. A static landmark";
        m_ch   = 0;

        *this = SemanticInfo(m_type, m_desc, m_ch);
        break;
      case 1:
        type   = "Leaf";
        m_desc = "Leaf from a vine trunk. A dynamic landmark";
        m_ch   = 1;

        *this = SemanticInfo(type, m_desc, m_ch);
        break;
      default:
        *this = SemanticInfo("Trunk", "Vine trunk", 0);
    }
  }

  std::string type;
  std::string description;
  int         character{};
};

struct SemanticFeature : public Feature {
  SemanticFeature() = default;
  // Class constructor
  // - initializes its pose, standard deviation and
  // - its sematic information
  SemanticFeature(const point&                  pos,
                  const Gaussian<point, point>& gauss,
                  const int&                    label)
  {
    (*this).pos   = pos;
    (*this).gauss = gauss;
    (*this).info  = SemanticInfo(label);
  }
  // Class constructor
  // - initializes its pose, standard deviation
  SemanticFeature(const point& pos, const Gaussian<point, point>& gauss)
  {
    (*this).pos   = pos;
    (*this).gauss = gauss;
  }

  // Print semantic landmark information
  void print()
  {
    std::string c = (info.character == 0) ? "static" : "dynamic";

    std::cout << "Landmark " << std::endl;
    std::cout << "   type:        " << info.type << std::endl;
    std::cout << "   description: " << info.description << std::endl;
    std::cout << "   character:   " << c << std::endl;
    std::cout << "   position:    " << pos;
    std::cout << "   stdev:      [" << gauss.stdev.x << "," << gauss.stdev.y << "]"
              << std::endl;
  }

  Gaussian<point, point> gauss;
  SemanticInfo           info;
};

} // namespace vineslam