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
struct SemanticInfo
{
  SemanticInfo() = default;
  SemanticInfo(const std::string& type, const std::string& description, const int& character)
  {
    type_ = type;
    description_ = description;
    character_ = character;
  }

  // Initialize semantic information of feature to give
  // to the mapper class
  explicit SemanticInfo(const int& label)
  {
    std::string m_type;
    std::string m_desc;
    int m_ch;

    switch (label)
    {
      case 0:
        m_type = "Tiny Grape Bunch";
        m_desc = "Grape bunch in an early stage.";
        m_ch = 0;

        *this = SemanticInfo(m_type, m_desc, m_ch);
        break;
      case 1:
        type_ = "Trunk";
        m_desc = "Vineyard trunk.";
        m_ch = 1;

        *this = SemanticInfo(type_, m_desc, m_ch);
        break;
    }
  }

  std::string type_;
  std::string description_;
  int character_{};
};

struct SemanticFeature : public Feature
{
  SemanticFeature() = default;
  // Class constructor
  // - initializes its pose, standard deviation and
  // - its sematic information
  SemanticFeature(const Point& pos, const Gaussian<Point, Point>& gauss, const int& label)
  {
    pos_ = pos;
    gauss_ = gauss;
    info_ = SemanticInfo(label);
    label_ = label;
  }
  // Class constructor
  // - initializes its pose, standard deviation
  SemanticFeature(const Point& pos, const Gaussian<Point, Point>& gauss)
  {
    pos_ = pos;
    gauss_ = gauss;
  }

  // Print semantic landmark information
  void print()
  {
    std::string c = (info_.character_ == 0) ? "static" : "dynamic";

    std::cout << "Landmark " << std::endl;
    std::cout << "   type:        " << info_.type_ << std::endl;
    std::cout << "   description: " << info_.description_ << std::endl;
    std::cout << "   character:   " << c << std::endl;
    std::cout << "   position:    " << "[" << pos_.x_ << ", " << pos_.y_ << "]" << std::endl;
    std::cout << "   stdev:      [" << gauss_.stdev_.x_ << "," << gauss_.stdev_.y_ << "]" << std::endl;
  }

  Gaussian<Point, Point> gauss_;
  SemanticInfo info_;
  int label_;
};

}  // namespace vineslam