#include <iostream>
#include <cstdlib>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


void readTransformFromText(const std::string &file, Eigen::Matrix<float, 4, 4, Eigen::RowMajor> &transform )
	 {




	        std::ifstream infile(file.c_str());
	        std::stringstream buffer;

	        buffer << infile.rdbuf();
	        float temp;
	        std::vector<float> matrixElements;
	        while (buffer >> temp ) {
	        matrixElements.push_back(temp);
	      //  std::cout<<"temp: "<<temp<<"\n";

	        }

	       transform= Eigen::Map<Eigen::Matrix<float,4,4,Eigen::RowMajor> >(matrixElements.data());
         // transform=Eigen::Map<Eigen::Matrix4f >(matrixElements.data());



	      infile.close();



	}

// This function displays the help
void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

// This is the main function
int
main (int argc, char** argv)
{

  // Show help
  // if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
  //   showHelp (argv[0]);
  //   return 0;
  // }

  // // Fetch point cloud filename in arguments | Works with PCD and PLY files
  // std::vector<int> filenames;
  // bool file_is_pcd = false;

  // filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

  // if (filenames.size () != 1)  {
  //   filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  //   if (filenames.size () != 1) {
  //     showHelp (argv[0]);
  //     return -1;
  //   } else {
  //     file_is_pcd = true;
  //   }
  // }

  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

 
    if (pcl::io::loadPLYFile (argv[1], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
      
      return -1;
    }
  

	pcl::PLYWriter plyWriter;

  Eigen::Matrix<float, 4, 4, Eigen::RowMajor>  transform_1 ;
  //readTransformFromText("/home/hamit/Softwares/PointNetLK/experiments/trans.out", transform_1);
  readTransformFromText(argv[2], transform_1);

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  if (atoi(argv[3])==1) pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_1);
  else if (atoi(argv[3])==-1) pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_1.inverse());

  //pcl::io::savePLYFile ("test_ply.ply", *transformed_cloud);
  plyWriter.write(argv[4], *transformed_cloud, true, false);
  
  return 0;
}