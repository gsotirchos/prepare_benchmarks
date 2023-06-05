#include "prepare_benchmarks.h"
#include <Eigen/Core>
#include <boost/algorithm/hex.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <iostream>
#include <octomap/octomap.h>
#include <yaml-cpp/yaml.h>

namespace benchmarks {
    template<typename T>
    auto readFromYAML(T & entry, YAML::Node const & node) -> bool {
        try {
            entry = node.as<T>();
        } catch (std::exception const & exc) {
            std::cerr << exc.what() << '\n';
            return false;
        }

        return true;
    }


    auto decompressHex(const std::string & hex) -> std::vector<int8_t> {
        std::vector<int8_t> unhexed;
        boost::algorithm::unhex(hex, std::back_inserter(unhexed));

        std::vector<char> decompress;
        try {
            boost::iostreams::filtering_ostream fos;
            fos.push(boost::iostreams::zlib_decompressor());
            fos.push(boost::iostreams::back_inserter(decompress));

            for (const auto & elem : unhexed) {
                fos << elem;
            }
        } catch (std::exception const & exc) {
            std::cerr << exc.what() << '\n';
            return unhexed;
        }

        return std::vector<int8_t>{decompress.begin(), decompress.end()};
    }


    auto createTreeFromData(double resolution, std::string const & data) -> octomap::OcTree {
        auto tree = octomap::OcTree(resolution);

        auto const decompressed = decompressHex(data);
        std::stringstream datastream;
        datastream.write(
            reinterpret_cast<char const *>(decompressed.data()),
            static_cast<std::streamsize>(decompressed.size())
        );

        tree.readData(datastream);

        return tree;
    }


    auto getVoxelsFromLeaf(
        std::vector<Eigen::Vector3d> & voxels,
        octomap::OcTree::leaf_iterator const & leaf_itr,
        double const & resolution
    ) -> void {
        octomap::point3d const leaf_center = leaf_itr.getCoordinate();
        double const leaf_size = leaf_itr.getSize();
        int const num_divisions = std::ceil(leaf_itr.getSize() / resolution);

        // Iterate over all subdivided voxels in the leaf
        for (int i = 0; i < num_divisions; ++i) {
            for (int j = 0; j < num_divisions; ++j) {
                for (int k = 0; k < num_divisions; ++k) {
                    // Calculate the center of the occupied voxel
                    double const voxel_center_x =
                        leaf_center.x() - (leaf_size / 2.0) + (i + 0.5) * resolution;
                    double const voxel_center_y =
                        leaf_center.y() - (leaf_size / 2.0) + (j + 0.5) * resolution;
                    double const voxel_center_z =
                        leaf_center.z() - (leaf_size / 2.0) + (k + 0.5) * resolution;

                    voxels.emplace_back(voxel_center_x, voxel_center_y, voxel_center_z);
                }
            }
        }
    }


    auto getOccupiedVoxelsFromTree(octomap::OcTree const & tree, double occupancy_threshold)
        -> std::vector<Eigen::Vector3d> {
        std::vector<Eigen::Vector3d> occupied_voxels;
        double const & resolution = tree.getResolution();

        // Iterate over all leaves
        for (octomap::OcTree::leaf_iterator leaf_itr = tree.begin_leafs(),
                                            end = tree.end_leafs();
             leaf_itr != end;
             ++leaf_itr) {
            // Only store the voxels of occupied leaves
            if (leaf_itr->getOccupancy() >= occupancy_threshold) {
                int const num_divisions = std::ceil(leaf_itr.getSize() / resolution);

                if (num_divisions == 1) {
                    // If the leaf has the smallest possible size (== resolution)
                    // then store it's center coordinets directly
                    octomap::point3d const leaf_center = leaf_itr.getCoordinate();
                    occupied_voxels.emplace_back(
                        leaf_center.x(), leaf_center.y(), leaf_center.z()
                    );
                } else {
                    // Otherwise split the leaf's volume into voxels and store
                    // the center coordinates of those instead
                    (void)getVoxelsFromLeaf(occupied_voxels, leaf_itr, resolution);
                }
            }

            //// Print leaf info
            //std::cout << "depth: " << leaf_itr.getDepth() << '\n'
            //          << "occupancy: " << leaf_itr->getOccupancy() << '\n'
            //          << "log odds: " << leaf_itr->getLogOdds() << '\n'
            //          << "size: " << leaf_itr.getSize() << '\n'
            //          << "center coord.:  [" << leaf_itr.getCoordinate().x() << ", "
            //          << leaf_itr.getCoordinate().y() << ", " << leaf_itr.getCoordinate().z()
            //          << "]\n------------------\n";
        }

        return occupied_voxels;
    }


    auto readOccupiedVoxelsFromYAML(std::string const & file_path)
        -> std::vector<Eigen::Vector3d> {
        YAML::Node const & node = YAML::LoadFile(file_path);

        //Eigen::Vector3d position(3);
        //Eigen::Vector3d orientation(4);
        std::string data;
        double resolution = 0;

        //(void)readFromYAML(position, node["world"]["octomap"]["origin"]["position"]);
        //(void)readFromYAML(orientation, node["world"]["octomap"]["origin"]["orientation"]);
        (void)readFromYAML(data, node["world"]["octomap"]["octomap"]["data"]);
        (void)readFromYAML(resolution, node["world"]["octomap"]["octomap"]["resolution"]);

        //// Print data from yaml
        //std::cout << "position: " << '\n';
        //for (auto const & i : position) {
        //    std::cout << "  " << i << '\n';
        //}
        //std::cout << "orientation: " << '\n';
        //for (auto const & i : orientation) {
        //    std::cout << "  " << i << '\n';
        //}
        //std::cout << "data: " << data << '\n';
        //std::cout << "resolution: " << resolution << '\n';

        octomap::OcTree const & tree = createTreeFromData(resolution, data);
        double const occupancy_threshold = 0.5;

        std::vector<Eigen::Vector3d> occupied_voxels =
            getOccupiedVoxelsFromTree(tree, occupancy_threshold);

        //// Print occupied voxels
        //std::cout << "OCCUPIED VOXELS" << '\n';
        //for (auto const & voxel : occupied_voxels) {
        //    std::cout << voxel[0] << ", " << voxel[1] << ", " << voxel[2] << '\n';
        //}

        return occupied_voxels;
    }
}  // namespace benchmarks


//auto main() -> int {
//    std::string const file_path =
//        "/home/ubuntu/panda_benchmarks/box_panda/scene_sensed0001.yaml";
//    auto occupied_voxels = benchmarks::readOccupiedVoxelsFromYAML(file_path);
//}
