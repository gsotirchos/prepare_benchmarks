#include <Eigen/Core>
#include <octomap/octomap.h>
#include <yaml-cpp/yaml.h>


namespace benchmarks {
    // Stores data of type T from a field in a yaml file.
    template<typename T>
    auto readFromYAML(T & entry, YAML::Node const & node) -> bool;

    // Converts a hex sequence in a string to a vector of integers.
    auto decompressHex(const std::string & hex) -> std::vector<int8_t>;

    // Creates an OcTree from with a specified resolution and fills it with
    // specified data.
    auto createTreeFromData(double resolution, std::string const & data) -> octomap::OcTree;

    // Stores the center coordinates of the voxels in an OcTree leaf.
    auto getVoxelsFromLeaf(
        std::vector<Eigen::Vector3d> & voxels,
        octomap::OcTree::leaf_iterator const & leaf_itr,
        double const & resolution
    ) -> void;

    // Returns a vector with the center coordinates of the occupied voxels
    // from an OcTree.
    auto getOccupiedVoxelsFromTree(octomap::OcTree const & tree, double occupancy_threshold)
        -> std::vector<Eigen::Vector3d>;

    // Returns a vector with the center coordinates of the occupied voxels
    // from a planning scene message stored in a yaml file.
    auto readOccupiedVoxelsFromYAML(std::string const & file_path)
        -> std::vector<Eigen::Vector3d>;
}  // namespace benchmarks
