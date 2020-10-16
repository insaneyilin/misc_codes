#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

bool ParseBoostArgs(int argc, char **argv,
                    boost::program_options::variables_map *boost_args) {
  boost::program_options::options_description desc("Allowd options");

  desc.add_options()("input_file", boost::program_options::value<std::string>()->required(),
                     "input xyz file")("output_file", boost::program_options::value<std::string>()->required(),
                                       "ouput pcd file")("leaf_size", boost::program_options::value<float>()->default_value(0.01f),
                                                         "VoxelGrid Leaf Size")("help", "Print help messages");

  boost::program_options::parsed_options opt_parsed =
      boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(opt_parsed, *boost_args);
  // help message
  if (boost_args->count("help")) {
    std::cout << desc << '\n';
    return false;
  }
  boost::program_options::notify(*boost_args);
  return true;
}

bool LoadXYZFileToPointCloud(const std::string &input_file, pcl::PointCloud<pcl::PointXYZI> *point_cloud) {
  point_cloud->clear();
  std::ifstream ifs(input_file);
  if (!ifs.is_open()) {
    return false;
  }
  pcl::PointXYZI point;
  while (ifs >> point.x >> point.y >> point.z >> point.intensity) {
    point_cloud->push_back(point);
  }

  return true;
}

int main(int argc, char **argv) {
  boost::program_options::variables_map args;
  if (!ParseBoostArgs(argc, argv, &args)) {
    std::cerr << "failed to parse args!\n";
    return 1;
  }

  const std::string input_file = args["input_file"].as<std::string>();
  const std::string output_file = args["output_file"].as<std::string>();
  const float leaf_size = args["leaf_size"].as<float>();
  std::cout << "input_file: " << input_file
            << "\noutput_file: " << output_file << "\nleaf_size: " << leaf_size << '\n';

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  if (!LoadXYZFileToPointCloud(input_file, cloud.get())) {
    return 1;
  }
  std::cout << "input points num: " << cloud->size() << '\n';

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.filter(*cloud_filtered);
  std::cout << "downsampled points num: " << cloud_filtered->size() << '\n';

  pcl::PCDWriter writer;
  writer.write(output_file, *cloud_filtered);
  return 0;
}
