#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

bool SavePointCloudToXYZFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             const std::string& out_xyz_filepath) {
  const size_t num_points = cloud->size();
  std::ofstream ofs(out_xyz_filepath);
  for (uint32_t i = 0; i < num_points; i++) {
    auto& pt = (*cloud)[i];
    ofs << pt.x << " " << pt.y << " " << pt.z << '\n';
  }
  ofs.close();
  return true;
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Usage: " << argv[0]
              << " <input_pcd_file> <output_xyz_file>\n";
    return 1;
  }
  const std::string input_pcd_filepath(argv[1]);
  const std::string output_xyz_filepath(argv[2]);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_filepath, *cloud) == -1) {
    std::cerr << "Cannot read " << input_pcd_filepath << "\n";
    return 1;
  }
  SavePointCloudToXYZFile(cloud, output_xyz_filepath);

  return 0;
}
