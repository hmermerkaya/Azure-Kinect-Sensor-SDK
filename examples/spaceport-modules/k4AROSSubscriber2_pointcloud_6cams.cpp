/*
 * k4AROSSubscriber2_pointcloud_6cams.cpp
 *
 *  Created on: Jul 6, 2021
 *      Author: hamit
 */




#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <algorithm>

#include <boost/range/algorithm.hpp>
#include <boost/program_options.hpp>
#include <boost/circular_buffer.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>


#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "3DContainer.h"
#include "k4a/k4a.h"
#include <signal.h>

bool is_done=false;
boost::mutex io_mutex;
static std::map<std::string, cv::Mat> mapSerialCalibs;
static std::map<std::string, cv::Mat> mapSerialDepthXYTables;
static std::map<std::string, int> mapSerials;
static std::map<std::string, k4a_calibration_t> k4aCalibs_;

static void convertMat2K4ACalibs(const std::map<std::string,cv::Mat> &calibs, std::map<std::string, k4a_calibration_t> &k4aCalibs) {
	 	//std::cout<<"mapSerialCalibs size "<<calibs.size()<<std::endl;
		for (const auto & calib: calibs) {

			memcpy(&k4aCalibs[calib.first].color_camera_calibration.extrinsics.rotation[0], calib.second.ptr<float>(0), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].color_camera_calibration.extrinsics.translation[0], calib.second.ptr<float>(1), 3*sizeof(float));
		    k4aCalibs[calib.first].color_camera_calibration.intrinsics.parameter_count = (unsigned int)calib.second.at<float>(2,0);
			memcpy(&k4aCalibs[calib.first].color_camera_calibration.intrinsics.parameters.v[0], calib.second.ptr<float>(3), 15*sizeof(float));
			k4aCalibs[calib.first].color_camera_calibration.intrinsics.type =  (k4a_calibration_model_type_t )calib.second.at<float>(4,0);
			k4aCalibs[calib.first].color_camera_calibration.metric_radius = calib.second.at<float>(5,0);
			k4aCalibs[calib.first].color_camera_calibration.resolution_height = (int)calib.second.at<float>(6,0);
			k4aCalibs[calib.first].color_camera_calibration.resolution_width = (int)calib.second.at<float>(7,0);

			k4aCalibs[calib.first].color_resolution = (k4a_color_resolution_t)calib.second.at<float>(8,0);

			memcpy(&k4aCalibs[calib.first].depth_camera_calibration.extrinsics.rotation[0], calib.second.ptr<float>(9), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].depth_camera_calibration.extrinsics.translation[0], calib.second.ptr<float>(10), 3*sizeof(float));
		    k4aCalibs[calib.first].depth_camera_calibration.intrinsics.parameter_count = (unsigned int)calib.second.at<float>(11,0);
			memcpy(&k4aCalibs[calib.first].depth_camera_calibration.intrinsics.parameters.v[0], calib.second.ptr<float>(12), 15*sizeof(float));
			k4aCalibs[calib.first].depth_camera_calibration.intrinsics.type =  (k4a_calibration_model_type_t )calib.second.at<float>(13,0);
			k4aCalibs[calib.first].depth_camera_calibration.metric_radius = calib.second.at<float>(14,0);
			k4aCalibs[calib.first].depth_camera_calibration.resolution_height = (int)calib.second.at<float>(15,0);
			k4aCalibs[calib.first].depth_camera_calibration.resolution_width = (int)calib.second.at<float>(16,0);

			k4aCalibs[calib.first].depth_mode = (k4a_depth_mode_t)calib.second.at<float>(17,0);

			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[0], calib.second.ptr<float>(18), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[0], calib.second.ptr<float>(19), 3*sizeof(float));

			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_ACCEL].rotation[0], calib.second.ptr<float>(20), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_ACCEL].translation[0], calib.second.ptr<float>(21), 3*sizeof(float));


			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_DEPTH].rotation[0], calib.second.ptr<float>(22), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_DEPTH].translation[0], calib.second.ptr<float>(23), 3*sizeof(float));

			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_ACCEL].rotation[0], calib.second.ptr<float>(24), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_ACCEL].translation[0], calib.second.ptr<float>(25), 3*sizeof(float));

			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR].rotation[0], calib.second.ptr<float>(26), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR].translation[0], calib.second.ptr<float>(27), 3*sizeof(float)); 

			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[0], calib.second.ptr<float>(28), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation[0], calib.second.ptr<float>(29), 3*sizeof(float));


		}

	
	
}



long getMicrotime(){
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	return currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;
}
typedef std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> >  vecMatrix4f;
template<typename K, typename V>
bool findByValue(K & vec, std::map<K, V> mapOfElemen, V value)
{
	bool bResult = false;
	auto it = mapOfElemen.begin();
	// Iterate through the map
	while(it != mapOfElemen.end())
	{
		// Check if value of this entry matches with given value
		if(it->second == value)
		{
			// Yes found
			bResult = true;
			// Push the key in given map
			vec=it->first;
		}
		// Go to next entry in map
		it++;
	}
	return bResult;
}





class CloudAndColorBuffer
{
  public:
    CloudAndColorBuffer () {}

    bool
    pushBack(const std::vector<std::pair<cv::Mat, cv::Mat > >   &);

    std::vector<std::pair<cv::Mat, cv::Mat > >
    getFront (); //

    inline bool
    isFull ()
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      return (buffer_.full ());
    }

    inline bool
    isEmpty ()
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      return (buffer_.empty ());
    }

    inline int
    getSize ()
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      return (int (buffer_.size ()));
    }

    inline int
    getCapacity ()
    {
      return (int (buffer_.capacity ()));
    }

    inline void
    setCapacity (int buff_size)
    {
      boost::mutex::scoped_lock buff_lock (bmutex_);
      buffer_.set_capacity (buff_size);
    }

  private:
    CloudAndColorBuffer (const CloudAndColorBuffer&); // Disabled copy constructor
    CloudAndColorBuffer& operator = (const CloudAndColorBuffer&); // Disabled assignment operator

    boost::mutex bmutex_;
    boost::condition_variable buff_empty_;


    boost::circular_buffer<std::vector<std::pair<cv::Mat, cv::Mat > >  >  buffer_;


};

//////////////////////////////////////////////////////////////////////////////////////////
bool
CloudAndColorBuffer::pushBack (const std::vector<std::pair<cv::Mat, cv::Mat > > & cloudColors)

{
  bool retVal = false;
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    if (!buffer_.full ()) retVal = true;
      buffer_.push_back(cloudColors);

  }
  buff_empty_.notify_one ();
  return (retVal);
}

//////////////////////////////////////////////////////////////////////////////////////////

std::vector<std::pair<cv::Mat, cv::Mat > >
CloudAndColorBuffer::getFront ()
{
   std::vector<std::pair<cv::Mat, cv::Mat > >  cloudColors;
  {
    boost::mutex::scoped_lock buff_lock (bmutex_);
    while (buffer_.empty ())
    {
      if (is_done)
        break;
//      {
 //          boost::mutex::scoped_lock io_lock (io_mutex);
//        cerr << "No data in buffer_ yet or buffer is empty." << endl;
//      }
      buff_empty_.wait (buff_lock);
    }
    //cloud = buffer_.front ();
    cloudColors=buffer_.front();
    buffer_.pop_front ();
  }
  return (cloudColors);
}

#define FPS_CALC(_WHAT_, buff) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    std::cout<<"buff.getSize () "<<buff.getSize () <<std::endl;\
    if (now - last >= 1.0) \
    { \
      cerr << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz. Queue size: " << buff.getSize () << "\n"; \
      count = 0; \
      last = now; \
    } \
}while(false)




class Producer
{
  private:
		CloudAndColorBuffer  &buf_;

		boost::shared_ptr<boost::thread> thread_;
		boost::mutex  mt;
		boost::condition_variable data_ready_cond_;;
		boost::mutex prod_mutex;

		const vecMatrix4f &transMatrices_;
		//std::map<std::string, int> mapSerials_;
		std::map<std::string, std::pair<cv::Mat,cv::Mat>> mapSerialDepthLookups, mapSerialColorLookups;
		//std::map<std::string, cv::Mat> mapSerialDepthXYTables;
		//std::map<std::string, cv::Mat> mapSerialCalibs;

		std::vector< cv::Mat> clouds;
		std::vector<cv::Mat> colors;
 		pcl::PLYWriter plyWriter;
		typedef std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string, std::string,
		std::string, std::string, std::string, std::string  >> stringTupleVector;

		stringTupleVector topicsDepthAndColorCamInfos_;
		int cloudAndImageCnt;
		int serials_size;
 		const bool useExact, useCompressed;

 		std::vector<bool> updateCloudAndImage;

 		bool save;
 		bool running;
 		size_t frame;
 		const size_t queueSize;

 		cv::Mat color, depth;
 		std::vector<cv::Mat >  camerasMatrixColor, camerasMatrixDepth;
 		cv::Mat lookupX, lookupY;

 //		typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
 //		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

// 		typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactSyncPolicy;
// 		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximateSyncPolicy;

 		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image > ApproximateSyncPolicy3;
 		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image , sensor_msgs::Image> ApproximateSyncPolicy4;


	 //	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image > ExactSyncPolicy;
 	//	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproximateSyncPolicy;

 		// 		typedef std::vector<sensor_msgs::Image>  vecsensor;
// 		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vecsensor> ApproximateVecSyncPolicy;
// 		typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, vecsensor> ExactVecSyncPolicy;

 		ros::NodeHandle nh;
 		ros::AsyncSpinner spinner;

 		image_transport::ImageTransport it;
 		std::vector<boost::shared_ptr<image_transport::SubscriberFilter > > subDepthImages;
 		std::vector<boost::shared_ptr<image_transport::SubscriberFilter> > subColorImages;
 		std::vector< boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy3> >> syncsApproximate;

 	    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy3> > syncDepthApproximate;
 	    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy4> > syncDepthApproximate1;

 	    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy3> > syncColorApproximate;
 	 	boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy4> > syncColorApproximate1;

// 		boost::shared_ptr<message_filters::Synchronizer<ExactVecSyncPolicy> >  syncDepthExactVec;
// 	    boost::shared_ptr<message_filters::Synchronizer<ApproximateVecSyncPolicy> > syncDepthApproximateVec;

 		//	std::vector< boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>>  subCameraSInfoColor, subCameraSInfoDepth;
 		std::vector< sensor_msgs::CameraInfo::ConstPtr> subCamerasInfoColor, subCamerasInfoDepth;
        std::vector< sensor_msgs::Image::ConstPtr> sub_xy_tables;
        std::vector< sensor_msgs::Image::ConstPtr> sub_calibs;
 	//	std::vector<boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo > >> subCameraSInfoColor, subCameraSInfoDepth;

 		pcl::PCDWriter writer;
 		std::ostringstream oss;
 		std::vector<int> params;

		bool ready = false;
		bool depthsOk = false, depths1Ok=false;
		bool colorsOk = false, colors1Ok = false;
		int cnt=0;
    ///////////////////////////////////////////////////////////////////////////////////////
    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
     {
			cv_bridge::CvImageConstPtr pCvImage;
			pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
			pCvImage->image.copyTo(image);




     }

    static void createCloud(const cv::Mat &depth, const cv::Mat & lookupX, const cv::Mat & lookupY, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
       {
         const float badPoint = std::numeric_limits<float>::quiet_NaN();
         #pragma omp parallel for
         for(int r = 0; r < depth.rows; ++r)
         {
		   pcl::PointXYZ *itP = &cloud->points[r * depth.cols];
           const uint16_t *itD = depth.ptr<uint16_t>(r);

        //   const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
           const float y = lookupY.at<float>(0, r);
           const float *itX = lookupX.ptr<float>();
           for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itX)
           {

			 register const float depthValue = *itD / 1000.0f;

          //   std::cout<<" depth "<<depthValue<<std::endl;
             // Check for invalid measurements
             if(*itD == 0)
             {
               // not valid
               itP->x = itP->y = itP->z =  badPoint;
              // itP->rgba = 0;
               continue;
             }
             itP->z = depthValue;
             itP->x = *itX * depthValue;
             itP->y = y * depthValue;
   //          itP->b = itC->val[0];
   //          itP->g = itC->val[1];
   //          itP->r = itC->val[2];
   //          itP->a = 255;
           }
         }
       }

   static void createCloud3(const cv::Mat &depth, const cv::Mat & xy_table, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
       {
         const float badPoint = std::numeric_limits<float>::quiet_NaN();
         #pragma omp parallel for
         for(int r = 0; r < depth.rows; ++r)
         {
		   pcl::PointXYZ *itP = &cloud->points[r * depth.cols];
           const uint16_t *itD = depth.ptr<uint16_t>(r);

        //   const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
         //  const float y = xy_table.at<cv::Vec2f>(0, r)[1];
         //  const float *itX = lookupX.ptr<float>();
           for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD )
           {

			 register const float depthValue = *itD / 1000.0f;

          //   std::cout<<" depth "<<depthValue<<std::endl;
             // Check for invalid measurements
			 const float x = xy_table.at<cv::Vec2f>(r, c)[0];
			 const float y = xy_table.at<cv::Vec2f>(r, c)[1];
             if(*itD == 0 && !isnan(x) && !isnan(y))
             {
               // not valid
               itP->x = itP->y = itP->z =  badPoint;
              // itP->rgba = 0;
               continue;
             }
             itP->z = depthValue;
             itP->x = x * depthValue;
             itP->y = y * depthValue;
   //          itP->b = itC->val[0];
   //          itP->g = itC->val[1];
   //          itP->r = itC->val[2];
   //          itP->a = 255;
           }
         }
       }
    static void createCloud2(const cv::Mat &depth, const cv::Mat & lookupX, const cv::Mat & lookupY, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
          {
           const float badPoint = std::numeric_limits<float>::quiet_NaN();
//
//            #pragma omp parallel for
//            for(int r = 0; r < depth.rows; ++r)
//            {
//              pcl::PointXYZ *itP = &cloud->points[r * depth.cols];
//              const uint16_t *itD = depth.ptr<uint16_t>(r);
//
//           //   const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
//              const float y = lookupY.at<float>(0, r);
//              const float *itX = lookupX.ptr<float>();
//
//              for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itX)
//              {
//                register const float depthValue = *itD / 1000.0f;
//               // std::cout<<" depth "<<depthValue<<std::endl;
//                // Check for invalid measurements
//                if(*itD == 0)
//                {
//                  // not valid
//                  itP->x = itP->y = itP->z =  badPoint;
//                 // itP->rgba = 0;
//                  continue;
//                }
//                itP->z = depthValue;
//                itP->x = *itX * depthValue;
//                itP->y = y * depthValue;
//      //          itP->b = itC->val[0];
//      //          itP->g = itC->val[1];
//      //          itP->r = itC->val[2];
//      //          itP->a = 255;
//              }
//            }
              const std::size_t w = depth.cols;
              const std::size_t h = depth.rows;
              const float * itD0 = (float *) depth.ptr();
          //  const char * itRGB0 = (char *) tmp_itRGB0.ptr();

           		pcl::PointXYZ * itP = &cloud->points[0];
                bool is_dense = true;

           		for(std::size_t y = 0; y < h; ++y){

           			const unsigned int offset = y * w;
           			const float * itD = itD0 + offset;
           			//const char * itRGB = itRGB0 + offset * 4;
           			const float dy = lookupY.at<float>(0,y);

           			for(std::size_t x = 0; x < w; ++x, ++itP, ++itD)
           			{
           				const float depth_value = *itD / 1000.0f;

           				if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){

           					const float rx = lookupX.at<float>(0,x) * depth_value;
                           	const float ry = dy * depth_value;
           					itP->z = depth_value;
           					itP->x = rx;
           					itP->y = ry;

//           					itP->b = itRGB[0];
//           					itP->g = itRGB[1];
//           					itP->r = itRGB[2];
           				} else {
           					itP->z =badPoint ;
           					itP->x = badPoint;
           					itP->y = badPoint;

//           					itP->b = qnan_;
//           					itP->g = qnan_;
//           					itP->r = qnan_;
           					is_dense = false;
            				}
           			}
           		}

           		cloud->is_dense = is_dense;


          }


    void createCloud(const cv::Mat &depth, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
       {
         const float badPoint = std::numeric_limits<float>::quiet_NaN();

         #pragma omp parallel for
         for(int r = 0; r < depth.rows; ++r)
         {
           pcl::PointXYZ *itP = &cloud->points[r * depth.cols];
           const uint16_t *itD = depth.ptr<uint16_t>(r);

        //   const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
           const float y = lookupY.at<float>(0, r);
           const float *itX = lookupX.ptr<float>();

           for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itX)
           {
             register const float depthValue = *itD / 1000.0f;
            // std::cout<<" depth "<<depthValue<<std::endl;
             // Check for invalid measurements
             if(*itD == 0)
             {
               // not valid
               itP->x = itP->y = itP->z = badPoint;
              // itP->rgba = 0;
               continue;
             }
             itP->z = depthValue;
             itP->x = *itX * depthValue;
             itP->y = y * depthValue;
   //          itP->b = itC->val[0];
   //          itP->g = itC->val[1];
   //          itP->r = itC->val[2];
   //          itP->a = 255;
           }
         }
       }
     void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
     {

    	//std::cout<<"caminfo width and height "<<cameraInfo->width<<std::endl;
		 cameraMatrix =cv::Mat::zeros(3, 3, CV_64F);
		double *itC = cameraMatrix.ptr<double>(0, 0);
		for(size_t i = 0; i < 9; ++i, ++itC)
		{
		 *itC = cameraInfo->K[i];
	// std::cout<<"itC " <<*itC<<std::endl;
		}

     //  std::cout<<cameraMatrix<<std::endl;
     }

     cv::Mat readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo) const
     {
   	 cv::Mat  cameraMatrix= cv::Mat::zeros(3, 3, CV_64F);
       double *itC = cameraMatrix.ptr<double>(0, 0);
       for(size_t i = 0; i < 9; ++i, ++itC)
       {
         *itC = cameraInfo->K[i];
       }

       return cameraMatrix;
     }



     inline std::string stripStrTillEnd(const std::string & str, const std::string &delim ) {

        	  std::size_t found = str.find_last_of(delim);
        	  return str.substr(found+1);
     }

     void createLookup(size_t width, size_t height, const cv::Mat & cameraMat, cv::Mat &lookupX, cv::Mat &lookupY)
     {
       const float fx = 1.0f / cameraMat.at<double>(0, 0);
       const float fy = 1.0f / cameraMat.at<double>(1, 1);
       const float cx = cameraMat.at<double>(0, 2);
       const float cy = cameraMat.at<double>(1, 2);
     //  *pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
       float *it;

       lookupY = cv::Mat(1, height, CV_32F);
       it = lookupY.ptr<float>();
       for(size_t r = 0; r < height; ++r, ++it)
       {
         *it = (r - cy + 0.5) * fy;
       }

       lookupX = cv::Mat(1, width, CV_32F);
       it = lookupX.ptr<float>();
       for(size_t c = 0; c < width; ++c, ++it)
       {
         *it = (c - cx + 0.5) * fx;
       }
     }




     void createLookup(size_t width, size_t height, const cv::Mat & cameraMat)
       {
         const float fx = 1.0f / cameraMat.at<double>(0, 0);
         const float fy = 1.0f / cameraMat.at<double>(1, 1);
         const float cx = cameraMat.at<double>(0, 2);
         const float cy = cameraMat.at<double>(1, 2);
       //  std::cout<<"cam params: "<<fx<<" "<<fy<<" "<<cx<<" "<<cy<<std::endl;


         float *it;

         lookupY = cv::Mat(1, height, CV_32F);
         it = lookupY.ptr<float>();
         for(size_t r = 0; r < height; ++r, ++it)
         {
           *it = (r - cy) * fy;
         }

         lookupX = cv::Mat(1, width, CV_32F);
         it = lookupX.ptr<float>();
         for(size_t c = 0; c < width; ++c, ++it)
         {
           *it = (c - cx) * fx;
         }
       }




//     void callback2(const sensor_msgs::Image::ConstPtr imageDepth1, const sensor_msgs::Image::ConstPtr imageColor1 )
//       {

//     	cv::Mat depth1;
// 	    cv::Mat color1;
// 		readImage(imageDepth1, depth1);
// 		readImage(imageColor1, color1);
// 	//	std::cout<<"header depth: "<<imageDepth1->header.frame_id<<std::endl;
// //		std::cout<<"header color: "<<imageColor1->header.frame_id<<std::endl;

// 		std::string serial= stripStrTillEnd(imageDepth1->header.frame_id, "_");

// 		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
// 		cloud->header.stamp = imageDepth1->header.stamp.toNSec();

// 		cloud->height = depth1.rows;
// 		cloud->width = depth1.cols;
// 		cloud->is_dense = false;
// 		cloud->points.resize(cloud->height * cloud->width);

// 		//std::unique_lock<std::mutex> lk(prod_mutex);

// 		{
// 			boost::mutex::scoped_lock lk(prod_mutex);


// 			createCloud(depth1, mapSerialDepthLookups[serial].first, mapSerialDepthLookups[serial].second, cloud);

// 			//std::cout<<"cloud pointer "<<clouds.at(mapSerials_[serial]).get()<<std::endl;
// 		//	writer.writeBinary(serial+"_"+std::to_string(imageDepth1->header.stamp.toSec()) +".pcd", *cloud);

// 			clouds.at(mapSerials_[serial]) = cloud;
// 			colors.at(mapSerials_[serial]) = color1 ;


// 			cloudAndImageCnt++;
// 		}

// 		 //lk.unlock();


// 		if  ( cloudAndImageCnt == serials_size/2 )  data_ready_cond_.notify_one();


//       }


    void callback3(const sensor_msgs::Image::ConstPtr imageDepth1, const sensor_msgs::Image::ConstPtr imageDepth2, const sensor_msgs::Image::ConstPtr imageDepth3 )
          {


       	 	std::vector<cv::Mat> depths(3);
       	 	readImage(imageDepth1, depths[0]);
          	readImage(imageDepth2, depths[1]);
        	readImage(imageDepth3, depths[2]);
        // 	readImage(imageDepth4, depths[3]);


         //   std::cout<<"depths "<<depths[0].rows<<" "<<depths[1].rows<<std::endl;
           //std::cout<<"header depth callback3: "<<imageDepth1->header.stamp<<" "<< imageDepth2->header.stamp<<" "<<imageDepth3->header.stamp<<std::endl;
        //   std::cout<<"header color: "<<imageColor1->header.stamp<<" "<< imageColor2->header.stamp<<std::endl;


         	std::vector<std::string> serials(3);
    		serials[0]= stripStrTillEnd(imageDepth1->header.frame_id, "_");
    		serials[1]= stripStrTillEnd(imageDepth2->header.frame_id, "_");
    		serials[2]= stripStrTillEnd(imageDepth3->header.frame_id, "_");
    	//	serials[3]= stripStrTillEnd(imageDepth4->header.frame_id, "_");

			std::vector<uint64_t> stamps(3);
    		stamps[0]=imageDepth1->header.stamp.toNSec();
    		stamps[1]=imageDepth2->header.stamp.toNSec();
    		stamps[2]=imageDepth3->header.stamp.toNSec();
    		//stamps[3]=imageDepth4->header.stamp.toNSec();

//           std::cout<<"serials "<<serial1<< " "<<serial2<<std::endl;
//           std::cout<<"serials index "<<mapSerials_[serial1]<< " "<<mapSerials_[serial2]<<std::endl;
    		//pcl::PointCloud<pcl::PointXYZ>::Ptr jkl = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());
    	//	for (int j=0; j<4; j++) clds.push_back(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>()));
//
    		{
    	       	boost::mutex::scoped_lock lk(prod_mutex);


				for ( int i =0; i< serials.size(); i++)  	clouds.at(mapSerials[serials[i]]) = depths[i];


				depthsOk=true;

    		}

    	   /*if (depthsOk && depths1Ok && colorsOk && colors1Ok)*/
    		data_ready_cond_.notify_one();
        //  std::cout<<"data ready "<< std::endl;

        }








    void callback4(const sensor_msgs::Image::ConstPtr imageColor1, const sensor_msgs::Image::ConstPtr imageColor2, const sensor_msgs::Image::ConstPtr imageColor3)
             {


          	 	std::vector<cv::Mat> clrs(3);
          	 	readImage(imageColor1, clrs[0]);
             	readImage(imageColor2, clrs[1]);
           	     readImage(imageColor3, clrs[2]);
            	// readImage(imageColor4, clrs[3]);


            //   std::cout<<"depths "<<depth1.rows<<" "<<depth2.rows<<std::endl;
             //  std::cout<<"header depth: "<<imageDepth1->header.stamp<<" "<< imageDepth4->header.stamp<<std::endl;
              //std::cout<<"header color2: "<<imageColor1->header.stamp<<" "<< imageColor2->header.stamp<<" "<<imageColor3->header.stamp<<std::endl;


				std::vector<std::string> serials(3);
				serials[0]= stripStrTillEnd(imageColor1->header.frame_id, "_");
				serials[1]= stripStrTillEnd(imageColor2->header.frame_id, "_");
			//	std::cout<<"serials "<<serials[0]<<" "<<serials[1]<<std::endl;
				 serials[2]= stripStrTillEnd(imageColor3->header.frame_id, "_");
				// serials[3]= stripStrTillEnd(imageColor4->header.frame_id, "_");



//				std::vector<uint64_t> stamps(4);
//				stamps[0]=imageColor1->header.stamp.toNSec();
//				stamps[1]=imageColor2->header.stamp.toNSec();
//				stamps[2]=imageColor3->header.stamp.toNSec();
//				stamps[3]=imageColor4->header.stamp.toNSec();



       		{
       	      	boost::mutex::scoped_lock lk(prod_mutex);

       	        for ( int i =0; i< serials.size(); i++) {
       	        	colors.at(mapSerials[serials[i]]) = clrs[i] ;

       	        }

       	        colorsOk=true;
       		}

       	   data_ready_cond_.notify_one();


           }

        void callback5(const sensor_msgs::Image::ConstPtr imageDepth1, const sensor_msgs::Image::ConstPtr imageDepth2, const sensor_msgs::Image::ConstPtr imageDepth3,  const sensor_msgs::Image::ConstPtr imageDepth4 )
          {


       	 	std::vector<cv::Mat> depths(3);
       	 	readImage(imageDepth2, depths[0]);
          	readImage(imageDepth3, depths[1]);
        	readImage(imageDepth4, depths[2]);
        // 	readImage(imageDepth4, depths[3]);


         //   std::cout<<"depths "<<depths[0].rows<<" "<<depths[1].rows<<std::endl;
          // std::cout<<"header depth callback5: "<<imageDepth2->header.stamp<<" "<< imageDepth3->header.stamp<<" "<<imageDepth4->header.stamp<<std::endl;
        //   std::cout<<"header color: "<<imageColor1->header.stamp<<" "<< imageColor2->header.stamp<<std::endl;


         	std::vector<std::string> serials(3);
    		serials[0]= stripStrTillEnd(imageDepth2->header.frame_id, "_");
    		serials[1]= stripStrTillEnd(imageDepth3->header.frame_id, "_");
    		serials[2]= stripStrTillEnd(imageDepth4->header.frame_id, "_");
    	//	serials[3]= stripStrTillEnd(imageDepth4->header.frame_id, "_");

			std::vector<uint64_t> stamps(3);
    		stamps[0]=imageDepth2->header.stamp.toNSec();
    		stamps[1]=imageDepth3->header.stamp.toNSec();
    		stamps[2]=imageDepth4->header.stamp.toNSec();
    		//stamps[3]=imageDepth4->header.stamp.toNSec();

//           std::cout<<"serials "<<serial1<< " "<<serial2<<std::endl;
//           std::cout<<"serials index "<<mapSerials_[serial1]<< " "<<mapSerials_[serial2]<<std::endl;
    		//pcl::PointCloud<pcl::PointXYZ>::Ptr jkl = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());
    	//	for (int j=0; j<4; j++) clds.push_back(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>()));
//
    		{
    	       	boost::mutex::scoped_lock lk(prod_mutex);


				for ( int i =0; i< serials.size(); i++)  	clouds.at(mapSerials[serials[i]]) = depths[i];


				depths1Ok=true;

    		}

    	   /*if (depthsOk && depths1Ok && colorsOk && colors1Ok)*/
    		data_ready_cond_.notify_one();
        //  std::cout<<"data ready "<< std::endl;

        }

    void callback6(const sensor_msgs::Image::ConstPtr imageColor1, const sensor_msgs::Image::ConstPtr imageColor2, const sensor_msgs::Image::ConstPtr imageColor3 , const sensor_msgs::Image::ConstPtr imageColor4)
             {


          	 	std::vector<cv::Mat> clrs(3);
          	 	readImage(imageColor2, clrs[0]);
             	readImage(imageColor3, clrs[1]);
           	    readImage(imageColor4, clrs[2]);
            	// readImage(imageColor4, clrs[3]);


            //   std::cout<<"depths "<<depth1.rows<<" "<<depth2.rows<<std::endl;
          //     std::cout<<"header depth: "<<imageDepth1->header.stamp<<" "<< imageDepth4->header.stamp<<std::endl;
            //  std::cout<<"header color: "<<imageColor1->header.stamp<<" "<< imageColor2->header.stamp<<" "<<imageColor3->header.stamp<<std::endl;


				std::vector<std::string> serials(3);
				serials[0]= stripStrTillEnd(imageColor2->header.frame_id, "_");
				serials[1]= stripStrTillEnd(imageColor3->header.frame_id, "_");
			//	std::cout<<"serials "<<serials[0]<<" "<<serials[1]<<std::endl;
				 serials[2]= stripStrTillEnd(imageColor4->header.frame_id, "_");
				// serials[3]= stripStrTillEnd(imageColor4->header.frame_id, "_");



//				std::vector<uint64_t> stamps(4);
//				stamps[0]=imageColor1->header.stamp.toNSec();
//				stamps[1]=imageColor2->header.stamp.toNSec();
//				stamps[2]=imageColor3->header.stamp.toNSec();
//				stamps[3]=imageColor4->header.stamp.toNSec();



       		{
       	      	boost::mutex::scoped_lock lk(prod_mutex);

       	        for ( int i =0; i< serials.size(); i++) {
       	        	colors.at(mapSerials[serials[i]]) = clrs[i] ;

       	        }

       	        colors1Ok=true;
       		}

       	   data_ready_cond_.notify_one();


           }


    void createTopicDepthandImagesCamInfosTuple(const std::vector<std::string>  &serials) {


		for(auto & s: serials ) {

  	  		topicsDepthAndColorCamInfos_.push_back(std::make_tuple( "/k4A_" + s + "/hd/image_depth",
  	  																"/k4A_" + s + "/hd/image_ir",
																	"/k4A_" + s + "/hd/image_color",
																	"/k4A_" + s + "/hd/depth_camera_info",
																	"/k4A_" + s + "/hd/ir_camera_info",
																	"/k4A_" + s + "/hd/color_camera_info",
																	"/k4A_" + s + "/hd/xy_table",
																	"/k4A_" + s + "/hd/transformed_image_color",
																	"/k4A_" + s + "/hd/point_cloud_image",
																	"/k4A_" + s + "/hd/calibration"
																	
																	)
  	  		);


  	}

    }

    std::string get_str_between_two_str(const std::string &s,
            const std::string &start_delim,
            const std::string &stop_delim)
    {
        unsigned first_delim_pos = s.find(start_delim);
        unsigned end_pos_of_first_delim = first_delim_pos + start_delim.length();
        unsigned last_delim_pos = s.find(stop_delim);
        last_delim_pos=s.find(stop_delim, last_delim_pos+1) ;

        return s.substr(end_pos_of_first_delim,
                last_delim_pos - end_pos_of_first_delim);
    }

    void createMapSerials(const std::vector<std::string> &serials){
    	size_t i = 0;
    	for(const auto & s:serials)  {
    		mapSerials.insert(std::make_pair(s,i++));
    	}

    }



  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Producer (CloudAndColorBuffer &buf, const std::vector<std::string>  &serials,
    		const bool useExact, const bool useCompressed, const vecMatrix4f &transMatrices)
      : buf_ (buf), useExact(useExact), useCompressed(useCompressed), transMatrices_(transMatrices), serials_size(serials.size()),
	      save(false), running(false), frame(0), queueSize(30),
	      nh("~"), spinner(0), it(nh)
	  {

    		createTopicDepthandImagesCamInfosTuple(serials);
    		createMapSerials(serials);


    	    camerasMatrixColor.resize(serials_size);
			camerasMatrixDepth.resize(serials_size);

			cloudAndImageCnt=0;

            clouds.resize(serials_size);
            colors.resize(serials_size);

			params.push_back(cv::IMWRITE_JPEG_QUALITY);
			params.push_back(100);
			params.push_back(cv::IMWRITE_PNG_COMPRESSION);
			params.push_back(1);
			params.push_back(cv::IMWRITE_PNG_STRATEGY);
			params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
			params.push_back(0);

			thread_.reset (new boost::thread (boost::bind (&Producer::start, this)));
	  }



    bool createPointCloudsfromContainerPNG(const std::string & filename) {

       	ThreeDContainer container;
       	bool result = container.openForRead((char *)filename.c_str());
       	if (!result) {
       		cout << "Cannot open file" << endl;
       		return false;
       	}
       	char* data;
       	int length;

       	int t = 0;
       	int textureCount = 0;
       	int streamIndex = -1;

      	int framecnt0=0, camcnt0=0;
       	int framecnt1=0, camcnt1=0;
       	while (true) {
       		streamIndex = container.read(data, length, OBJ_FORMAT);
       		if (streamIndex == -1) {
       			break;
       		}
       		if (streamIndex == 0) {
       			if (camcnt0%6 == 0 &&  camcnt0!=0) {
                    framecnt0++;
                    camcnt0=0;
       			}


       			cv::Mat tmpDepth(1024, 1024, CV_16UC1, data);

       			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    //   			cloud->header.stamp = imageDepth1->header.stamp.toNSec();

    			cloud->height = tmpDepth.rows;
    			cloud->width = tmpDepth.cols;
    			cloud->is_dense = false;
    			cloud->points.resize(cloud->height * cloud->width);
    			std::string serial;

    		//	bool found = findByValue<std::string, int>(serial, mapSerials_, camcnt0);
//			    auto it = std::find_if(std::begin(mapSerials_), std::end(mapSerials_),
//									   [&camcnt0](const std::pair<string, int> &p) { return p.second == camcnt0; });
			    for (std::map<string, int>::const_iterator it = mapSerials.begin(); it != mapSerials.end(); ++it) {
			      if (it->second == camcnt0)  {
			    	  serial = it->first;
			    	  break;
			      }
			    }

			  //  std::cout<<"serial #: "<<serial<<" "<<camcnt0<<std::endl;

       		//	createCloud(tmpDepth, mapSerialDepthLookups[serial].first, mapSerialDepthLookups[serial].second, cloud);
				createCloud3(tmpDepth, mapSerialDepthXYTables[serial], cloud);

      			try {
					 if (camcnt0!=0) pcl::transformPointCloud ( *cloud, *cloud, transMatrices_.at(camcnt0-1) );
				   //   std::cout<<"transform : \n"<<transMatrices_.at(i);
				  }
				  catch (const std::out_of_range& e) {
					  cout << "Out of Range error.";
				  }

       			//string name = "Debug/recovered_" + to_string(framecnt0) + "_"+to_string(camcnt0)+ ".pcd";
       	     	//pcl::io::savePCDFile( name, *cloud, true ); // Binary format

       			string name = "Debug/recovered_" + to_string(framecnt0) + "_"+to_string(camcnt0)+ ".ply";
       		//	pcl::io::savePLYFileBinary( name, *cloud ); // Binary format
       			plyWriter.write(name, *cloud, true, false);
       			camcnt0++;


       		}
       		else if (streamIndex == 1) {

       			if (camcnt1%6 == 0 && camcnt1!=0 ) {
       			   		   		   			framecnt1++;
       			   		   		   			camcnt1=0;
       			   			}
       			ofstream ofs;
       			string name ="Debug/recovered_" + to_string(framecnt1) + "_"+to_string(camcnt1)+ ".jpg";
       			ofs.open(name);
       			ofs.write(data, length);
       			ofs.close();
       			camcnt1++;

       		}
       	}
       	container.close();
       	return true;
    }

////////////////////////////////////////////////////////////////////////////

    void
      start ()
      {

  			image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");


  			for (const auto & t:topicsDepthAndColorCamInfos_) {
  				subDepthImages.push_back( boost::make_shared<image_transport::SubscriberFilter>(it, std::get<0>(t), queueSize, hints) );
  				subColorImages.push_back( boost::make_shared<image_transport::SubscriberFilter>(it, std::get<2>(t), queueSize, hints) );

  			//	subCameraSInfoDepth.push_back( boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, std::get<2>(t) , queueSize));
  			//	subCameraSInfoColor.push_back( boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, std::get<3>(t), queueSize));
  				subCamerasInfoDepth.push_back( ros::topic::waitForMessage<sensor_msgs::CameraInfo>(std::get<3>(t),nh) );
  				subCamerasInfoColor.push_back( ros::topic::waitForMessage<sensor_msgs::CameraInfo>(std::get<5>(t),nh) );
                sub_xy_tables.push_back(ros::topic::waitForMessage<sensor_msgs::Image>(std::get<6>(t),nh))  ;
                sub_calibs.push_back(ros::topic::waitForMessage<sensor_msgs::Image>(std::get<9>(t),nh))  ;
             //std::cout<<"tuple componets "<< std::get<0>(t)<<" "<<std::get<1>(t)<<std::endl;
  			}



				std::transform (subCamerasInfoDepth.begin(), subCamerasInfoDepth.end(), std::inserter(mapSerialDepthLookups, mapSerialDepthLookups.end()),

						[this](sensor_msgs::CameraInfo::ConstPtr camInfo )  {


							cv::Mat tmpCamInfo;
							readCameraInfo(camInfo, tmpCamInfo);
						//	std::cout<<"caminfo width and height "<<camInfo->width<<std::endl;
							cv::Mat tmplookupX, tmplookupY;
							//createLookup(static_cast<size_t>(camInfo->width), static_cast<size_t>(camInfo->height), tmpCamInfo,
									//tmplookupX, tmplookupY );
							createLookup(1024, 1024, tmpCamInfo, tmplookupX, tmplookupY );
							std::string serial= stripStrTillEnd(camInfo->header.frame_id, "_");
							std::cout<<"serial "<<serial<<std::endl;
							return std::make_pair(serial, std::make_pair(tmplookupX, tmplookupY ));


				});

                std::transform (sub_xy_tables.begin(), sub_xy_tables.end(), std::inserter(mapSerialDepthXYTables, mapSerialDepthXYTables.end()),
					[this](sensor_msgs::Image::ConstPtr xy_table) {
					cv::Mat mat_xy_table;
					readImage(xy_table, mat_xy_table);
					//std::cout <<"mat_xy_table rows: "<<mat_xy_table.rows;
					std::string serial= stripStrTillEnd(xy_table->header.frame_id, "_");
					//std::cout<<"serial "<<serial<<std::endl;
					std::cout<<"xy_tables values "<<mat_xy_table.at<cv::Vec2f>(564, 545)[0]<<" "<< mat_xy_table.at<cv::Vec2f>(564, 545)[2]<<std::endl;
					return std::make_pair(serial, mat_xy_table);




					});

                std::transform (sub_calibs.begin(), sub_calibs.end(), std::inserter(mapSerialCalibs, mapSerialCalibs.end()),
					[this](sensor_msgs::Image::ConstPtr calib) {
					cv::Mat mat_calib;
					readImage(calib, mat_calib);
					//std::cout <<"mat_calib rows: "<<mat_calib.rows;
					std::string serial= stripStrTillEnd(calib->header.frame_id, "_");
					//std::cout<<"serial "<<serial<<std::endl;
					//std::cout<<"calib values "<<mat_calibs.at<cv::Vec2f>(564, 545)[0]<<" "<< mat_xy_table.at<cv::Vec2f>(564, 545)[2]<<std::endl;
					return std::make_pair(serial, mat_calib);




					});

				//std::cout<<"mapSerialCalibs size "<<mapSerialCalibs.size()<<std::endl;
				convertMat2K4ACalibs(mapSerialCalibs, k4aCalibs_);

			//	std::cout<<"1 step"<<std::endl;
				syncDepthApproximate =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy3>>(ApproximateSyncPolicy3(queueSize),
						  *subDepthImages[0], *subDepthImages[1] , *subDepthImages[2]);
				syncDepthApproximate->registerCallback(boost::bind(&Producer::callback3, this, _1, _2,_3));
                syncDepthApproximate1 =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy4>>(ApproximateSyncPolicy4(queueSize),
						  *subDepthImages[2], *subDepthImages[3] , *subDepthImages[4], *subDepthImages[5]);
				syncDepthApproximate1->registerCallback(boost::bind(&Producer::callback5, this, _1, _2,_3, _4));
			//	std::cout<<"2 step"<<std::endl;
				syncColorApproximate =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy3>>(ApproximateSyncPolicy3(queueSize),
						  *subColorImages[0], *subColorImages[1], *subColorImages[2] );
				syncColorApproximate->registerCallback(boost::bind(&Producer::callback4, this, _1, _2, _3));
                syncColorApproximate1 =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy4>>(ApproximateSyncPolicy4(queueSize),
						  *subColorImages[2], *subColorImages[3], *subColorImages[4], *subColorImages[5] );
				syncColorApproximate1->registerCallback(boost::bind(&Producer::callback6, this, _1, _2, _3,_4));

				//first set of 4 cameras
				// if(useExact)
				// {
				// 	syncDepthExact =  boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
				// 		  *subDepthImages[0], *subDepthImages[1], *subDepthImages[2], *subDepthImages[3] );
				// 	syncDepthExact->registerCallback(boost::bind(&Producer::callback3, this, _1, _2, _3, _4));
				// }
				// else
				// {
				// 	syncDepthApproximate =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
				// 		  *subDepthImages[0], *subDepthImages[1], *subDepthImages[2], *subDepthImages[3] );
				// 	syncDepthApproximate->registerCallback(boost::bind(&Producer::callback3, this, _1, _2, _3, _4));
				// //	syncDepthApproximate->registerCallback(boost::bind(&Producer::callback3, this, _1, _2, _3, _4), this);

				// }



				// if(useExact)
				// {
				// 	syncColorExact =  boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
				// 		  *subColorImages[0], *subColorImages[1], *subColorImages[2], *subColorImages[3] );
				// 	syncColorExact->registerCallback(boost::bind(&Producer::callback4, this, _1, _2, _3, _4));
				// }
				// else
				// {
				// 	syncColorApproximate =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
				// 		  *subColorImages[0], *subColorImages[1], *subColorImages[2], *subColorImages[3] );
				// 	syncColorApproximate->registerCallback(boost::bind(&Producer::callback4, this, _1, _2, _3, _4));


				// }

				// //second set of 4 cameras

				// if(useExact)
				// {
				// 	syncDepthExact1 =  boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
				// 			*subColorImages[2], *subDepthImages[3], *subDepthImages[4], *subDepthImages[5] );
				// 	syncDepthExact1->registerCallback(boost::bind(&Producer::callback5, this, _1, _2, _3, _4));
				// }
				// else
				// {
				// 	syncDepthApproximate1 =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
				// 			*subColorImages[2],*subDepthImages[3], *subDepthImages[4], *subDepthImages[5] );
				// 	syncDepthApproximate1->registerCallback(boost::bind(&Producer::callback5, this, _1, _2, _3 , _4));


				// }

				// if(useExact)
				// {
				// 	syncColorExact1 =  boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
				// 			*subColorImages[2], *subColorImages[3], *subColorImages[4], *subColorImages[5] );
				// 	syncColorExact1->registerCallback(boost::bind(&Producer::callback6, this, _1, _2, _3, _4));
				// }
				// else
				// {
				// 	syncColorApproximate1 =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
				// 			*subColorImages[2], *subColorImages[3], *subColorImages[4], *subColorImages[5] );
				// 	syncColorApproximate1->registerCallback(boost::bind(&Producer::callback6, this, _1, _2, _3 , _4));


				// }

//				// set of 6 cameras
//				if(useExact)
//				{
//					syncDepthExact =  boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
//						  *subDepthImages[0], *subDepthImages[1], *subDepthImages[2], *subDepthImages[3] , *subDepthImages[4], *subDepthImages[5]);
//					syncDepthExact->registerCallback(boost::bind(&Producer::callback7, this, _1, _2, _3, _4, _5, _6));
//				}
//				else
//				{
//					syncDepthApproximate =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
//						  *subDepthImages[0], *subDepthImages[1], *subDepthImages[2], *subDepthImages[3], *subDepthImages[4], *subDepthImages[5]);
//					syncDepthApproximate->registerCallback(boost::bind(&Producer::callback7, this, _1, _2, _3, _4, _5, _6));
//
//
//				}
//
//
//
//				if(useExact)
//				{
//					syncColorExact =  boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
//						  *subColorImages[0], *subColorImages[1], *subColorImages[2], *subColorImages[3], *subDepthImages[4], *subDepthImages[5] );
//					syncColorExact->registerCallback(boost::bind(&Producer::callback8, this, _1, _2, _3, _4, _5, _6));
//				}
//				else
//				{
//					syncColorApproximate =  boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
//						  *subColorImages[0], *subColorImages[1], *subColorImages[2], *subColorImages[3],  *subDepthImages[4], *subDepthImages[5] );
//					syncColorApproximate->registerCallback(boost::bind(&Producer::callback8, this, _1, _2, _3, _4, _5, _6));
//
//
//				}


//




//				std::vector<image_transport::SubscriberFilter  >  imageVec {((subDepthImages.begin()+1)), ((subDepthImages.end()))};
//
//				if(useExact)
//				{
//					syncDepthExactVec =  boost::make_shared<message_filters::Synchronizer<ExactVecSyncPolicy>>(ExactVecSyncPolicy(queueSize),
//							*(*(subDepthImages.begin())), imageVec );
//					syncDepthExactVec->registerCallback(boost::bind(&Producer::callback5, this, _1, _2));
//				}
//				else
//				{
//					syncDepthApproximateVec =   boost::make_shared<message_filters::Synchronizer<ApproximateVecSyncPolicy>>(ApproximateVecSyncPolicy(queueSize),
//							*(*(subDepthImages.begin())), imageVec );
//					syncDepthApproximateVec->registerCallback(boost::bind(&Producer::callback5, this, _1, _2));
//
//
//				}


//				for (size_t i = 1; i < subDepthImages.size() ; ++i) {
//
//						 if(useExact)
//								{
//								  syncsExact.push_back(boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
//										  *subDepthImages[0], *subDepthImages[i], *subColorImages[0], *subColorImages[i]
//										//  *subCameraSInfoDepth[0], *subCameraSInfoDepth[i], *subCameraSInfoColor[0], *subCameraSInfoColor[i]
//								  ));
//								  syncsExact.back()->registerCallback(boost::bind(&Producer::callback, this, _1, _2, _3, _4));
//								}
//								else
//								{
//								  syncsApproximate.push_back(boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
//										  *subDepthImages[0], *subDepthImages[i], *subColorImages[0], *subColorImages[i]
//										//  *subCameraSInfoDepth[0], *subCameraSInfoDepth[i], *subCameraSInfoColor[0], *subCameraSInfoColor[i]
//								  ));
//								  syncsApproximate.back()->registerCallback(boost::bind(&Producer::callback, this, _1, _2, _3, _4));
//								}
//
//
//					}
//



//  			for (size_t i = 0; i < subDepthImages.size() ; ++i) {
//
//  				if(useExact)
//  								{
//  								  syncsExact.push_back(boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
//  										  *subDepthImages[i],  *subColorImages[i]
//
//  								  ));
//  								  syncsExact.back()->registerCallback(boost::bind(&Producer::callback2, this, _1, _2));
//  								}
//  								else
//  								{
//  								  syncsApproximate.push_back(boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
//  										  *subDepthImages[i],  *subColorImages[i]
//
//  								  ));
//  								  syncsApproximate.back()->registerCallback(boost::bind(&Producer::callback2, this, _1, _2 ));
//  								}
//
//  			}
//



  			spinner.start();

 		    std::chrono::milliseconds duration(1);
//std::cout<<"2.5 step"<<std::endl;

			while(1)
			{
			  if(  !ros::ok())


			  {
				return;
			  }


			    	 {
						boost::mutex::scoped_lock lk(prod_mutex);
						//std::cout<<"2.6 step"<<std::endl;
						data_ready_cond_.wait(lk, [this] { return ( depthsOk && colorsOk && depths1Ok  && colors1Ok ); });
						//std::cout<<"2.7 step"<<std::endl;
					//	data_ready_cond_.wait(lk, [this] { return ( depthsOk && colorsOk  ); });
					//	data_ready_cond_.wait(lk);

						depthsOk = false;depths1Ok = false;
						colorsOk = false;colors1Ok = false;

						std::vector<std::pair<cv::Mat, cv::Mat > > colorClouds;
					//	std::cout<<"color size: "<<clouds.size()<<std::endl;
						for(int i=0;i<clouds.size(); i++)  colorClouds.push_back(std::make_pair(clouds[i], colors[i]));
//						for(int i=0;i<clouds.size(); i++){
//
//						  colorClouds.push_back(std::make_pair(boost::make_shared<pcl::PointCloud<PointT>>(*clouds[i]), colors[i].clone()));
//						}

						if (!buf_.pushBack ( colorClouds) )
						{
						  {
							boost::mutex::scoped_lock io_lock (io_mutex);
							pcl::console::print_warn ("Warning! Buffer was full, overwriting data!\n");
						  }
						}
						FPS_CALC ("cloud callback.", buf_);

			     }
			  if (is_done )	break;

			  cnt++;
			 // if  (cnt == 3) is_done =true;
			  std::this_thread::sleep_for(duration);
			}





      }
    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      spinner.stop();
      thread_->join ();
      boost::mutex::scoped_lock io_lock (io_mutex);
      pcl::console::print_highlight ("Producer done.\n");
    }








};


class Consumer
{
  private:
    ///////////////////////////////////////////////////////////////////////////////////////

	void write2Disk (  std::vector<std::pair<cv::Mat, cv::Mat > >   &&colorClouds )
    {

      std::string timestamp = boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ());

      std::vector<uchar> buff;//buffer for coding
	  std::vector<int> param(2);
	  param[0] = cv::IMWRITE_JPEG_QUALITY;
	  param[1] = 80;


      for (int i=0;i<colorClouds.size();++i) {


            size_t size =colorClouds[i].first.total() * colorClouds[i].first.elemSize();

    	    if (!container.writeTextureToContainer((char*) colorClouds[i].first.data , size, t*100000, STREAM_INDEX_MESH_TEXTURE)) {
    	 					cout << "Cannot write texture to containerr" << endl;
    	 					//return false;
    	 	    }
    	    t++;

            cv::imencode(".jpg",  colorClouds[i].second , buff);
            size = buff.size();//colorClouds[i].second.total() * colorClouds[i].second.elemSize();
            if (!container.writeTextureToContainer((char*) buff.data(), size, t*100000, STREAM_INDEX_SPACE_CONTAINER_TEXTURE)) {
                                cout << "Cannot write texture to container" << endl;
                                //return false;
                }
            t++;






     }



      FPS_CALC ("cloud write.", buf_);
    }
	
 static void convertMat2K4ACalibs(const std::map<std::string,cv::Mat> &calibs, std::map<std::string, k4a_calibration_t> &k4aCalibs) {
	 	std::cout<<"mapSerialCalibs size "<<calibs.size()<<std::endl;
		for (const auto & calib: calibs) {

			//std::cout<<"rotation matttttttttttttttttttt: "<<calib.second.ptr<float>(0)[0]<<std::endl;
			memcpy(&k4aCalibs[calib.first].color_camera_calibration.extrinsics.rotation[0], calib.second.ptr<float>(0), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].color_camera_calibration.extrinsics.translation[0], calib.second.ptr<float>(1), 3*sizeof(float));
		    k4aCalibs[calib.first].color_camera_calibration.intrinsics.parameter_count = (unsigned int)calib.second.at<float>(2,0);
			memcpy(&k4aCalibs[calib.first].color_camera_calibration.intrinsics.parameters.v[0], calib.second.ptr<float>(3), 15*sizeof(float));
			k4aCalibs[calib.first].color_camera_calibration.intrinsics.type =  (k4a_calibration_model_type_t )calib.second.at<float>(4,0);
			k4aCalibs[calib.first].color_camera_calibration.metric_radius = calib.second.at<float>(5,0);
			k4aCalibs[calib.first].color_camera_calibration.resolution_height = (int)calib.second.at<float>(6,0);
			k4aCalibs[calib.first].color_camera_calibration.resolution_width = (int)calib.second.at<float>(7,0);

			k4aCalibs[calib.first].color_resolution = (k4a_color_resolution_t)calib.second.at<float>(8,0);

			memcpy(&k4aCalibs[calib.first].depth_camera_calibration.extrinsics.rotation[0], calib.second.ptr<float>(9), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].depth_camera_calibration.extrinsics.translation[0], calib.second.ptr<float>(10), 3*sizeof(float));
		    k4aCalibs[calib.first].depth_camera_calibration.intrinsics.parameter_count = (unsigned int)calib.second.at<float>(11,0);
			memcpy(&k4aCalibs[calib.first].depth_camera_calibration.intrinsics.parameters.v[0], calib.second.ptr<float>(12), 15*sizeof(float));
			k4aCalibs[calib.first].depth_camera_calibration.intrinsics.type =  (k4a_calibration_model_type_t )calib.second.at<float>(13,0);
			k4aCalibs[calib.first].depth_camera_calibration.metric_radius = calib.second.at<float>(14,0);
			k4aCalibs[calib.first].depth_camera_calibration.resolution_height = (int)calib.second.at<float>(15,0);
			k4aCalibs[calib.first].depth_camera_calibration.resolution_width = (int)calib.second.at<float>(16,0);

			k4aCalibs[calib.first].depth_mode = (k4a_depth_mode_t)calib.second.at<float>(17,0);

			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[0], calib.second.ptr<float>(18), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[0], calib.second.ptr<float>(19), 3*sizeof(float));

			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_ACCEL].rotation[0], calib.second.ptr<float>(20), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_ACCEL].translation[0], calib.second.ptr<float>(21), 3*sizeof(float));


			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_DEPTH].rotation[0], calib.second.ptr<float>(22), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_DEPTH].translation[0], calib.second.ptr<float>(23), 3*sizeof(float));

			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_ACCEL].rotation[0], calib.second.ptr<float>(24), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_ACCEL].translation[0], calib.second.ptr<float>(25), 3*sizeof(float));

			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR].rotation[0], calib.second.ptr<float>(26), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR].translation[0], calib.second.ptr<float>(27), 3*sizeof(float)); 
			
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation[0], calib.second.ptr<float>(28), 9*sizeof(float));
			memcpy(&k4aCalibs[calib.first].extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation[0], calib.second.ptr<float>(29), 3*sizeof(float));


		}

	
	
}
void createTransfomedColor (const cv::Mat &depth, const cv::Mat &color, const k4a_calibration_t &calib, cv::Mat &transformed_color){

	//cv::imwrite("color_before.jpg", color);
	//std::cout<<"color.type(): "<<color.type()<<std::endl;

	k4a_image_t k4aColor = NULL;

	//k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, color.cols, color.rows, color.cols * 4 * (int)sizeof(uint8_t), &k4aColor);

    k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_COLOR_BGRA32, color.cols, color.rows, color.cols *4* sizeof(uint8_t),
											(uint8_t*)color.data, color.cols* color.rows* 4* sizeof(uint8_t), NULL, NULL, &k4aColor);

	//std::cout<<"sizeof(cv::Vec4b) "<<sizeof(cv::Vec4b)<<std::endl;
	//memcpy(k4a_image_get_buffer(k4aColor), color.data, color.rows * color.cols * sizeof(cv::Vec4b));


    int     height1 = k4a_image_get_height_pixels(k4aColor);
    int    width1 = k4a_image_get_width_pixels(k4aColor);
    cv::Mat tmp_color(height1, width1, CV_8UC4, (void *)k4a_image_get_buffer(k4aColor));
    //cv::imwrite("color_after.jpg", tmp_color);


	k4a_image_t k4aDepth = NULL;

	k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, depth.cols, depth.rows, depth.cols  * (int)sizeof(uint16_t), &k4aDepth);

	memcpy(k4a_image_get_buffer(k4aDepth), &depth.ptr<uint16_t>(0)[0], depth.rows * depth.cols * sizeof(uint16_t));

	

	k4a_transformation_t transformation = NULL;
    transformation = k4a_transformation_create(&calib);

 	k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                     depth.cols,
                                                     depth.rows,
                                                     depth.cols* 4 * (int)sizeof(uint8_t),
                                                     &transformed_color_image))
 	{
           printf("Failed to create transformed color image\n");
            return;
	}


	if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation,
                                                                                  k4aDepth,
                                                                                  k4aColor,
                                                                                  transformed_color_image))
	{
		printf("Failed to compute transformed color image\n");
		return;
	}
    int height = k4a_image_get_height_pixels(transformed_color_image);
    int width = k4a_image_get_width_pixels(transformed_color_image);

    transformed_color = cv::Mat(height, width, CV_8UC4, (uint8_t *) k4a_image_get_buffer(transformed_color_image));
	//cv::imwrite("tranformed_color.jpg", transformed_color);

	if (transformation != NULL)
	{
		k4a_transformation_destroy(transformation);
	}

	k4a_image_release(k4aColor);
    k4a_image_release(k4aDepth);
	k4a_image_release(transformed_color_image);


}

void createCloudFromDepthRGB(const cv::Mat &depth, const cv::Mat & xy_table, const cv::Mat &transformed_color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud){
	
	     
		 const float badPoint = std::numeric_limits<float>::quiet_NaN();
         #pragma omp parallel for
         for(int r = 0; r < depth.rows; ++r)
         {
		   pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
           const uint16_t *itD = depth.ptr<uint16_t>(r);

           const cv::Vec4b *itC = transformed_color.ptr<cv::Vec4b>(r);
        
           for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC )
           {

			 register const float depthValue = *itD / 1000.0f;

             //std::cout<<" depth "<<depthValue<<std::endl;
             // Check for invalid measurements
			 const float x = xy_table.at<cv::Vec2f>(r, c)[0];
			 const float y = xy_table.at<cv::Vec2f>(r, c)[1];
             if(*itD == 0 || isnan(x) || isnan(y))
             {
               // not valid
               itP->x = itP->y = itP->z =  badPoint;
              // itP->rgba = 0;
               continue;
             }
             itP->z = depthValue;
             itP->x = x * depthValue;
             itP->y = y * depthValue;
			 itP->b = itC->val[0];
			 itP->g = itC->val[1];
			 itP->r = itC->val[2];
			  itP->a = 255;
           }
         }
	// std::cout<<" step 1 "<<std::endl;
	 
	 
	 
	 
	
	
}

 static void createCloud(const cv::Mat &cl, const cv::Mat & color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
   {
 
	 const float badPoint = std::numeric_limits<float>::quiet_NaN();
	 #pragma omp parallel for
	 for(int r = 0; r < cl.rows; ++r)
	 {
	   pcl::PointXYZRGB *itP = &cloud->points[r * cl.cols];
	   const cv::Vec3f *itCl = cl.ptr<cv::Vec3f>(r);
	   const cv::Vec4b *itCol = color.ptr<cv::Vec4b>(r);
	//   const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
	 //  const float y = xy_table.at<cv::Vec2f>(0, r)[1];
	 //  const float *itX = lookupX.ptr<float>();
	   for(size_t c = 0; c < (size_t)cl.cols; ++c, ++itP, ++itCl, ++itCol )
	   {

		 //register const float depthValue = *itD / 1000.0f;

	  //   std::cout<<" depth "<<depthValue<<std::endl;
		 // Check for invalid measurements
		 //const float x = xy_table.at<cv::Vec2f>(r, c)[0];
		 //const float y = xy_table.at<cv::Vec2f>(r, c)[1];
		 if(itCl->val[2] == 0 && !isnan(itCl->val[0]) && !isnan(itCl->val[1]))
		 {
		   // not valid
		   itP->x = itP->y = itP->z =  badPoint;
		   itP->rgb = 0;
		   continue;
		 }

		 itP->x = itCl->val[0];
		 itP->y = itCl->val[1];
		 itP->z = itCl->val[2];

		 itP->b = itCol->val[0];
		 itP->g = itCol->val[1];
		 itP->r = itCol->val[2];
		// itP->a = 255;
	   }
	 }
   }

template<typename PointT> 
void filterPointCloud(const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &cloud1 ) {

		pcl::PCLPointCloud2::Ptr cloud0 (new pcl::PCLPointCloud2);

		//pcl::toPCLPointCloud2(*cloud, *cloud0);
		pcl::toPCLPointCloud2(cloud, *cloud0);

//			pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
//
//			vox.setInputCloud(cloud0);
//
//			vox.setLeafSize(0.01f, 0.01f, 0.01f);
//
//			vox.filter(*cloud0);

//		    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
//			outrem.setInputCloud(cloud0);
//			outrem.setRadiusSearch(0.02);
//			outrem.setMinNeighborsInRadius (20);
//			//outrem.setKeepOrganized(false);
//			// apply filter
//			outrem.filter (*cloud0);

		pcl::PassThrough<pcl::PCLPointCloud2> pass;
		// pass.setInputCloud (cloud0);
		// pass.setFilterFieldName ("z");
		// pass.setFilterLimits (0.7, 2.2);
		// pass.setFilterLimitsNegative (false);
		// pass.setKeepOrganized (true);
		// pass.filter (*cloud0);


		pass.setInputCloud (cloud0);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (-0.6, 1);
		pass.setFilterLimitsNegative (false);
		pass.setKeepOrganized (true);
		pass.filter (*cloud0);
//
//
		pass.setInputCloud (cloud0);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (-0.7, 1.6);
		pass.setFilterLimitsNegative (false);
		pass.setKeepOrganized (true);
		pass.filter (*cloud0);

		pcl::fromPCLPointCloud2(*cloud0, cloud1);

}
 void write2Disk3 ( const  std::vector<std::pair<cv::Mat, cv::Mat > >   &depthColors ) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
		
	 for (int i=0;i<depthColors.size();++i)  {
		 
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
		cloud->height = depthColors[i].first.rows;
		cloud->width = depthColors[i].first.cols;
		cloud->is_dense = false;
		cloud->points.resize(cloud->height * cloud->width);
		
		std::string serial;
		for (std::map<string, int>::const_iterator it = mapSerials.begin(); it != mapSerials.end(); ++it) {
		  if (it->second == i)  {
			  serial = it->first;
			  break;
		  }
		}
		 cv::Mat transformedColor;
		 //cv::imwrite("Files/color_"+std::to_string(frame_cnt)+".jpg", depthColors[i].second);

		 createTransfomedColor(depthColors[i].first, depthColors[i].second, k4aCalibs_[serial], transformedColor);
		// cv::imwrite("Files/image_"+std::to_string(frame_cnt++)+".jpg", transformedColor);
		 createCloudFromDepthRGB(depthColors[i].first, mapSerialDepthXYTables[serial], transformedColor, cloud );

		try {
			if (i!=0) pcl::transformPointCloud ( *cloud, *cloud, transMatrices_.at(i-1) );
				//   std::cout<<"transform : \n"<<transMatrices_.at(i);
		}
		catch (const std::out_of_range& e) {
					cout << "Out of Range error.";
		}




		filterPointCloud(*cloud, *cloud);


		std::vector<int> ind;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, ind); 

       // std::string name = "Files/recovered_" + std::to_string(frame_cnt++) + ".ply";
		//std::string name = "Files/recovered_" + std::to_string(frame_cnt++) + ".pcd";
		//pcl::io::savePCDFileBinaryCompressed(name, *cloud);
       		//	pcl::io::savePLYFileBinary( name, *cloud ); // Binary format
        //plyWriter.write(name, *cloud, true, false);



		 *merged_cloud += *cloud;
	 }
	//std::vector<int> ind;
	//pcl::removeNaNFromPointCloud(*merged_cloud, *merged_cloud, ind); 

	std::string name = "Files/recovered_" + std::to_string(frame_cnt++) + ".ply";
	//std::string name = "Files/recovered_" + std::to_string(frame_cnt++) + ".pcd";
       		//	pcl::io::savePLYFileBinary( name, *cloud ); // Binary format
    plyWriter.write(name, *merged_cloud, true, false);
	//pcl::io::savePCDFileBinaryCompressed(name, *merged_cloud);
	FPS_CALC ("cloud write.", buf_);
	 
 }

   void write2Disk2 ( const  std::vector<std::pair<cv::Mat, cv::Mat > >   &colorClouds ) {
//   			cloud->header.stamp = imageDepth1->header.stamp.toNSec();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		 for (int i=0;i<colorClouds.size();++i)  {

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

			cloud->height = colorClouds[i].first.rows;
			cloud->width = colorClouds[i].first.cols;
			cloud->is_dense = false;
			cloud->points.resize(cloud->height * cloud->width);

			createCloud(colorClouds[i].first, colorClouds[i].second, cloud);

			try {
				if (i!=0) pcl::transformPointCloud ( *cloud, *cloud, transMatrices_.at(i) );
				   //   std::cout<<"transform : \n"<<transMatrices_.at(i);
			}
			catch (const std::out_of_range& e) {
					  cout << "Out of Range error.";
			}

			*merged_cloud += *cloud;







		}

		string name = "Files/recovered_" + std::to_string(frame_cnt++) + ".ply";
       		//	pcl::io::savePLYFileBinary( name, *cloud ); // Binary format
       	plyWriter.write(name, *merged_cloud, true, false);




   }


   void createDracoPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>  &cloud, draco::PointCloud &dracoPointCloud) {

		int nr_points  = cloud.width * cloud.height;
		dracoPointCloud.set_num_points(nr_points);

		draco::PointAttribute pa;
		pa.Init(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32, false, sizeof(float) * 3);
		int32_t pos_att_id = dracoPointCloud.AddAttribute(pa, false, nr_points);
		//draco :: PointAttribute * att_ptr = dracoPointCloud.attribute (pos_att_id);
		draco :: PointIndex pointIndex (0);

		draco::PointAttribute ca;
		ca.Init(draco::GeometryAttribute::COLOR, 3, draco::DT_INT8, false, sizeof(unsigned char)*3);
		int color_att_id = dracoPointCloud.AddAttribute(ca, false, nr_points);

		draco::PointIndex pointColor(0);

		for (const auto &point: cloud.points){
			std::vector<float> vertex_data(3);
			memcpy(vertex_data.data(), point.getArray3fMap().data(), sizeof(float)*3);
			std::cout<<"point xyz: "<<point.getArray3fMap()<<std::endl;
			std::vector<float> color_data(3);
			memcpy(color_data.data(), point.getRGBVector3i().data(), sizeof(unsigned char) *3);

			dracoPointCloud.attribute(pos_att_id)->SetAttributeValue(dracoPointCloud.attribute(pos_att_id)
					->mapped_index(pointIndex), vertex_data.data());

			dracoPointCloud.attribute(color_att_id)->SetAttributeValue(dracoPointCloud.attribute(color_att_id)
								->mapped_index(pointColor), color_data.data());

			pointIndex++;
			pointColor++;


		}



   }
    ///////////////////////////////////////////////////////////////////////////////////////
    // Consumer thread function
    void
    receiveAndProcess ()
    {

      while (true)
      {
        if (is_done)   break;
       // send2KinfuApp (buf_.getFront());
        write2Disk (buf_.getFront ());
      }

      {
        boost::mutex::scoped_lock io_lock (io_mutex);
        pcl::console::print_info ("Writing remaing %ld clouds in the buffer to disk...\n", buf_.getSize ());
      }
      while (!buf_.isEmpty ()) {

    	//send2KinfuApp (buf_.getFront());
       write2Disk (buf_.getFront ());
      }
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Consumer (CloudAndColorBuffer &buf, vecMatrix4f &transMatrices)
      : buf_ (buf), transMatrices_(transMatrices)
    {
  //    app=boost::shared_ptr<KinFuLSApp<pcl::PointXYZRGB>>(new KinFuLSApp<pcl::PointXYZRGB>());
     //  boost::this_thread::sleep (boost::posix_time::milliseconds (1000));

    	//ThreeDContainer container;
    //	bool isFileOpened = container.open((char*)"test_3d_video_ply_jpg.mp4", true);
	    //convertMat2K4ACalibs(mapSerialCalibs, k4aCalibs_);

        isFileOpened = container.open((char*)"test_3d_video_ply_jpg.mp4", true);
		
    	thread_.reset (new boost::thread (boost::bind (&Consumer::receiveAndProcess, this)));

  	    params.push_back(cv::IMWRITE_JPEG_QUALITY);
  		params.push_back(80);
  		params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  		params.push_back(9);
  		params.push_back(cv::IMWRITE_PNG_STRATEGY);
  		params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
  		params.push_back(0);

    }

    ///////////////////////////////////////////////////////////////////////////////////////
    void
    stop ()
    {
      thread_->join ();
      boost::mutex::scoped_lock io_lock (io_mutex);

	  container.close();

      pcl::console::print_highlight ("Consumer done.\n");
    }

  private:
    CloudAndColorBuffer &buf_;
    boost::shared_ptr<boost::thread> thread_;
    pcl::PCDWriter writer_;
    pcl::PLYWriter plyWriter;
    vecMatrix4f &transMatrices_;
    std::vector<int> params;
    ThreeDContainer container;
    bool isFileOpened;
    int t=0;
	int frame_cnt=0;
	//std::map<std::string, k4a_calibration_t> k4aCalibs_;

};




void
ctrlC (int)
{
  boost::mutex::scoped_lock io_lock (io_mutex);
  pcl::console::print_info ("\nCtrl-C detected, exit condition set to true.\n");
  is_done = true;
}


void produceTransforms(const std::string & folder) {
	DIR *dir;
	struct dirent *ent;
	std::vector<std::string> filenames;
	if ((dir = opendir (folder.c_str())) != NULL) {
	  /* print all the files and directories within directory */
	  while ((ent = readdir (dir)) != NULL) {
		 std::string tmpfilename=std::string(ent->d_name);
		  if (tmpfilename.find(".out") != std::string::npos) {
			  filenames.push_back(tmpfilename);
		  	  std::cout<<"filenames "<<tmpfilename<<std::endl;
		  }
	  }
	  closedir (dir);
	} else {
	  /* could not open directory */
	  perror ("");
	  return ;
	}

}

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


void produceTranformMatrices(const std::string &folder, vecMatrix4f & transformMatrices) {
	using namespace boost::filesystem;
	path p(folder);
	std::vector<std::string> filenames;

	try
	{
		if (exists(p))    // does p actually exist?
		{
			if (is_regular_file(p))        // is p a regular file?
			cout << p << " size is " << file_size(p) << '\n';

			else if (is_directory(p))      // is p a directory?
			{
			cout << p << " is a directory containing:\n";

			typedef std::vector<path> vec;             // store paths,
			vec v;                                // so we can sort them later

			copy(directory_iterator(p), directory_iterator(), back_inserter(v));

			sort(v.begin(), v.end());             // sort, since directory iteration
			vec::const_iterator it(v.begin()) ;

			for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
			{
						  if ((*it).string().find(".out") != std::string::npos) {
							  filenames.push_back((*it).string());
						  	  cout << "   " << *it << '\n';
						  }

			}




			}
			else
			  cout << p << " exists, but is neither a regular file nor a directory\n";
		}
		else
		cout << p << " does not exist\n";
	}

	catch (const filesystem_error& ex)
	{
	cout << ex.what() << '\n';
	}


	if (filenames.size() < 1 ) {
		std::cout<<"At least 1 files containing transformation matrix shoud exists \n";
		return ;
	}

	for (const auto & file:filenames) {
		Eigen::Matrix<float,4,4,Eigen::RowMajor> tmpMat;
		readTransformFromText(file, tmpMat );
		transformMatrices.push_back(tmpMat);
		std::cout<<"transform: \n"<<tmpMat<<std::endl;

	}




}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "azure_kinect");

	namespace po = boost::program_options;
	std::vector<std::string> serials;
	std::string serials_file;
	int postime;
	try {
		po::options_description desc("");
		desc.add_options()
		 ("serials,s", po::value<std::vector<std::string> >()->multitoken(), "kinectv2 serials")
		// ("postime", po::value<int>()->default_value(3), "the time to start grabber");
		 ("file_serial", po::value<std::string>(),"file containing serials");
		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);

		if (vm.count("file_serial") && vm.count("serials") ) throw po::error("both argument 'serials' and 'file_serials' are provided! Not possible!"); //std::cout<<"hellole "<<std::endl;
		else  if (vm.count("file_serial") && !vm.count("serials") )	serials_file = vm["file_serial"].as<std::string >();
		else  if (!vm.count("file_serial")   && vm.count("serials") ) serials = vm["serials"].as<std::vector<std::string> >();
		else throw po::error("Argument 'serials' or 'file_serials' must be provided!" );

	}
	catch (const po::error &ex)
	  {
	    std::cerr << ex.what() << '\n';
	 //   PCL_ERROR ("Any kinectv2 serial is not provided! Please enter serial to continue. Syntax is %s --serials serial1 serial2 ...\n", argv[0]);
	    exit(1);

	 }




	std::ifstream infile(serials_file);

	std::string serial;
	while (infile >> serial)
	{
	    serials.push_back(serial);
	//    std::cout<<"serial "<<serial<<std::endl;

	}


	if(!ros::ok())
	{
	return 0;
	}

	bool useExact=false;
	bool useCompressed=true;



	//produceTransforms("./");
    vecMatrix4f transformMatrices;

	produceTranformMatrices("./", transformMatrices);


//	Receiver receiver(serials , useExact, useCompressed);
////
//	receiver.start();
////
//	receiver.stop();


	const int buff_size = 400;


	CloudAndColorBuffer  buf;
	buf.setCapacity (buff_size);

	Producer producer (buf, serials , useExact, useCompressed, transformMatrices);
//	boost::this_thread::sleep (boost::posix_time::milliseconds (2000));
	//std::this_thread::sleep_for(std::chrono::milliseconds (7000));
	Consumer consumer (buf, transformMatrices);

	signal (SIGINT, ctrlC);
	producer.stop();
	consumer.stop ();
	ros::shutdown();
	//producer.createPointCloudsfromContainerPNG("test_3d_video_ply_jpg.mp4");

	//ros::shutdown();
	return 0;
	}

