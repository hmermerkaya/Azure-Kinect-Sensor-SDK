#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
 void
	 readTransformFromText(const std::string &file, Eigen::Matrix<float,4,4,Eigen::RowMajor>&transform )
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


int main(int argc, char **argv) {

    std::vector<std::string> files;
    std::string transFile, file1, file2;
    pcl::console::parse_argument(argc, argv,"-t", transFile);
    pcl::console::parse_argument(argc, argv, "-f1", file1);
    pcl::console::parse_argument(argc, argv, "-f2", file2);

   // std::cout<<"files "<<files[0]<< " "<< files[1]<<std::endl;
    if (argc<3) {

        std::cout<<"Usage -t transformation_file -f file1 file2. Exiting!!\n" ;

    }

    Eigen::Matrix<float,4,4,Eigen::RowMajor>  transform;
   //  Eigen::Matrix4f transform ;
    readTransformFromText(transFile, transform);
    std::cout<<transform<<std::endl;
   

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    std::cout<<"step 7"<<std::endl;
    

    // if (pcl::io::loadPCDFile<pcl::PointXYZ> (file1, *cloud1) == -1) //* load the file
    // {
    //     PCL_ERROR ("Couldn't read the file  \n");
    //     return (-1);
    // }


    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file2, *cloud2) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read the file  \n");
        return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
    std::cout<<"step 8"<<std::endl;
   
    pcl::transformPointCloud (*cloud2, *transformed_cloud2, transform);
    std::cout<<transform<<std::endl;
  //  pcl::io::savePCDFileASCII (file1, *cloud1);
    pcl::io::savePCDFileASCII ("trans"+file2, *transformed_cloud2);
   // std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

   //std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
   //viewer->setBackgroundColor (0, 0, 0);
   //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb1(cloud1);
   //viewer->addPointCloud<pcl::PointXYZ>(cloud1, rgb1, "cloud1");
   //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb2(cloud2);
   //viewer->addPointCloud<pcl::PointXYZ>(cloud2, rgb2, "cloud2");

   //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
   //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
  // while (!viewer->wasStopped ()) { // Display the visualiser until 'q' key is pressed
  //     viewer->spinOnce ();
  // }

    return 0;

}