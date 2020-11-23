#include "../include/localization_node.hpp"

int main(int argc, char** argv)
{
  vineslam::LocalizationNode vineslam_node(argc, argv);
  return 0;
}

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

LocalizationNode::LocalizationNode(int argc, char** argv)
{
}

LocalizationNode::~LocalizationNode() = default;

}  // namespace vineslam
