/*
 * rosPairKinectv2Viewer.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: hamit
 */



// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <boost/range/algorithm.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/condition_variable.hpp>

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


class Receiver
{
public:
  enum Mode
  {
	IMAGE = 0,
	CLOUD,
	BOTH
  };

private:
	std::mutex lock;
	std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string, std::string, std::string >> topicsIRAndColorCamInfos_;
     std::vector<std::string>  serials_;
	const bool useExact, useCompressed;
  boost::mutex mutex;
  boost::condition_variable data_ready_cond_;
  bool irOk=false, colorOk=false;
	bool updateImage, updateCloud, updateImage1, updateImage2;
	bool save;
	bool running;
	size_t frame;
	const size_t queueSize;
	std::string frame_id1, frame_id2, frame_id3, frame_id4, frame_id5, frame_id6, time_stamp1, time_stamp2;
  std::string frame_color_id1, frame_color_id2, frame_color_id3, frame_color_id4, frame_color_id5, frame_color_id6, time_color_stamp1, time_color_stamp2;

	cv::Mat color1, ir1, color2, ir2, color3, ir3, color4, ir4, color5, ir5, color6, ir6;
	std::vector<cv::Mat >  cameraSMatrixColor, cameraSMatrixDepth;
	pcl::PLYWriter plyWriter;
	cv::Mat lookupX, lookupY;
	//std::map<std::string, std::pair<cv::Mat,cv::Mat>> mapSerialDepthLookups;
	//std::map<std::string, std::pair<cv::Mat,cv::Mat>> mapSerialColorLookups;
    	std::map<std::string, cv::Mat> mapSerialDepthXYTables;
    	std::map<std::string, cv::Mat> mapSerialIRCamInfo;
    	std::map<std::string, cv::Mat> mapSerialColorCamInfo;
//	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image > ExactSyncPolicy;
//	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image > ApproximateSyncPolicy;

//	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
//	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

   	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ExactSyncPolicy;
   	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproximateSyncPolicy6;

	ros::NodeHandle nh;
	ros::AsyncSpinner spinner;
	// std::vector< image_transport::ImageTransport> it;

	image_transport::ImageTransport it;
	std::vector<boost::shared_ptr<image_transport::SubscriberFilter > > subIRImages;
	std::vector<boost::shared_ptr<image_transport::SubscriberFilter> > subColorImages;
	std::vector<boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy> > > syncsExact;
	std::vector< boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> >> syncsApproximate;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy6> > syncsApproximateColor;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy6> > syncsApproximateIR;
//	std::vector< boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>>  subCameraSInfoColor, subCameraSInfoDepth;
	std::vector< sensor_msgs::CameraInfo::ConstPtr> subCamerasInfoColor, subCamerasInfoIR;
  std::vector< sensor_msgs::Image::ConstPtr> sub_xy_tables;
//	std::vector<boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo > >> subCameraSInfoColor, subCameraSInfoDepth;



//
//  image_transport::ImageTransport it;
//  boost::shared_ptr<image_transport::SubscriberFilter >  subIRImages1;
//  boost::shared_ptr<image_transport::SubscriberFilter >  subIRImages2;
//
//  boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy> >  syncSExact;
//  boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > syncSApproximate;

  //std::thread imageViewerThread;
  Mode mode;

  cv::VideoWriter writer;
  std::ostringstream oss;
  std::vector<int> params;
 // int file_cnt=0;

public:
  Receiver(const std::vector<std::string>  &serials, const bool useExact, const bool useCompressed) // @suppress("Class members should be properly initialized")
    : useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(10), updateImage1(false), updateImage2(false),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {

	    serials_=serials;
        createTopicDepthandImagesCamInfosTuple(serials);
	    cameraSMatrixColor.resize(topicsIRAndColorCamInfos_.size());
		cameraSMatrixDepth.resize(topicsIRAndColorCamInfos_.size());
        cv::Size size(1280,720);
        writer.open("out.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, size);

		params.push_back(cv::IMWRITE_JPEG_QUALITY);
		params.push_back(100);
		params.push_back(cv::IMWRITE_PNG_COMPRESSION);
		params.push_back(9);
		params.push_back(cv::IMWRITE_PNG_STRATEGY);
		params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
		params.push_back(0);


  }

  ~Receiver()
  {
  }

  void run()
  {
    start();
    stop();
  }





  void start()
  {

	  running = true;



    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");


    for (const auto & t:topicsIRAndColorCamInfos_) {
	//	std::cout<<"get t0 "<<std::get<0>(t)<<std::endl;
    	// subIRImages.push_back( boost::make_shared<image_transport::SubscriberFilter>(it, std::get<0>(t), queueSize, hints) );
    	// subColorImages.push_back( boost::make_shared<image_transport::SubscriberFilter>(it, std::get<1>(t), queueSize, hints) );

    	// subCamerasInfoDepth.push_back( ros::topic::waitForMessage<sensor_msgs::CameraInfo>(std::get<2>(t),nh) );
    	// subCamerasInfoColor.push_back( ros::topic::waitForMessage<sensor_msgs::CameraInfo>(std::get<3>(t),nh) );

      subIRImages.push_back( boost::make_shared<image_transport::SubscriberFilter>(it, std::get<1>(t), queueSize, hints) );
      subColorImages.push_back( boost::make_shared<image_transport::SubscriberFilter>(it, std::get<2>(t), queueSize, hints) );

      //	subCameraSInfoDepth.push_back( boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, std::get<2>(t) , queueSize));
      //	subCameraSInfoColor.push_back( boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, std::get<3>(t), queueSize));
      subCamerasInfoIR.push_back( ros::topic::waitForMessage<sensor_msgs::CameraInfo>(std::get<4>(t),nh) );
      subCamerasInfoColor.push_back( ros::topic::waitForMessage<sensor_msgs::CameraInfo>(std::get<5>(t),nh) );
      sub_xy_tables.push_back(ros::topic::waitForMessage<sensor_msgs::Image>(std::get<6>(t),nh))  ;

    }




	std::transform (subCamerasInfoIR.begin(), subCamerasInfoIR.end(), std::inserter(mapSerialIRCamInfo,mapSerialIRCamInfo.end()),

  							[this](sensor_msgs::CameraInfo::ConstPtr camInfo )  {


  			  			    	cv::Mat tmpCamInfo;
  			  			    	readCameraInfo(camInfo, tmpCamInfo);

  			  			    	
  			  			    	std::string serial= stripStrTillEnd(camInfo->header.frame_id, "_");
  			  			    	//std::cout<<"serial "<<serial<<std::endl;
  			  			    	return std::make_pair(serial, tmpCamInfo);


  			});

	std::transform (subCamerasInfoColor.begin(), subCamerasInfoColor.end(), std::inserter(mapSerialColorCamInfo,mapSerialColorCamInfo.end()),

  							[this](sensor_msgs::CameraInfo::ConstPtr camInfo )  {


  			  			    	cv::Mat tmpCamInfo;
  			  			    	readCameraInfo(camInfo, tmpCamInfo);

  			  			    	
  			  			    	std::string serial= stripStrTillEnd(camInfo->header.frame_id, "_");
  			  			    	std::cout<<"serial: "<<serial<<std::endl;
  			  			    	return std::make_pair(serial, tmpCamInfo);


  			});


	std::transform (sub_xy_tables.begin(), sub_xy_tables.end(), std::inserter(mapSerialDepthXYTables, mapSerialDepthXYTables.end()),
					[this](sensor_msgs::Image::ConstPtr xy_table) {
					cv::Mat mat_xy_table;
					readImage(xy_table, mat_xy_table);
					//std::cout <<"mat_xy_table rows: "<<mat_xy_table.rows;
					std::string serial= stripStrTillEnd(xy_table->header.frame_id, "_");
					//std::cout<<"serial "<<serial<<std::endl;
				//	std::cout<<"xy_tables values "<<mat_xy_table.at<cv::Vec2f>(564, 545)[0]<<" "<< mat_xy_table.at<cv::Vec2f>(564, 545)[2]<<std::endl;
					return std::make_pair(serial, mat_xy_table);




					});



/*
    for (size_t i = 1; i < subIRImages.size() ; ++i) {

    	 if(useExact)
    	  	    {
    	  	      syncsExact.push_back(boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
    	  	    		  *subIRImages[0], *subIRImages[i], *subColorImages[0], *subColorImages[i]
    	  	      ));
    	  	      syncsExact.back()->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    	  	    }
    	  	    else
    	  	    {
    		      syncsApproximate.push_back(boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
    		    		  *subIRImages[0], *subIRImages[i], *subColorImages[0], *subColorImages[i]
    		      ));
    	  	      syncsApproximate.back()->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    	  	    }


    }
*/


    //  if(useExact)
    //                 {
    //                   syncsExact.push_back(boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(queueSize),
    //                               *subIRImages[0], *subIRImages[1], *subIRImages[2], *subIRImages[3],*subColorImages[0], *subColorImages[1], *subColorImages[2], *subColorImages[3]
    //                   ));
    //                   syncsExact.back()->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4, _5, _6, _7,_8));
    //                 }
    //                 else
    //                 {
    //                   syncsApproximate.push_back(boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(queueSize),
    //                               *subIRImages[0], *subIRImages[1], *subIRImages[2], *subIRImages[3],*subColorImages[0], *subColorImages[1], *subColorImages[2], *subColorImages[3]
    //                   ));
    //                   syncsApproximate.back()->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4, _5, _6, _7, _8));
    //                 }


    syncsApproximateIR =boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy6>>(ApproximateSyncPolicy6(queueSize),
                *subIRImages[0], *subIRImages[1], *subIRImages[2], *subIRImages[3],*subIRImages[4], *subIRImages[5]
    );
    syncsApproximateIR->registerCallback(boost::bind(&Receiver::callback1, this, _1, _2, _3, _4, _5, _6));

    syncsApproximateColor=boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy6>>(ApproximateSyncPolicy6(queueSize),
                                  *subColorImages[0], *subColorImages[1], *subColorImages[2], *subColorImages[3], *subColorImages[4], *subColorImages[5]
    );
    syncsApproximateColor->registerCallback(boost::bind(&Receiver::callback2, this, _1, _2, _3, _4, _5, _6));



    spinner.start();

    std::chrono::milliseconds duration(1);

  //   while(!updateImage1 && !updateImage2)
  //   {
  //     if(!ros::ok())
  //     {
  //       return;
  //     }

	// //  std::cout<<" ros ok"<<std::endl;



  //     std::this_thread::sleep_for(duration);
  //   }

    imageViewer();
  }

  void stop()
  {
    spinner.stop();
    running = false;

  }

private:

  


  inline std::string stripStrTillEnd(const std::string & str, const std::string &delim ) {

      	  std::size_t found = str.find_last_of(delim);
      	  return str.substr(found+1);
   }


  void callback(const sensor_msgs::Image::ConstPtr imageIR1, const sensor_msgs::Image::ConstPtr imageIR2 ,const sensor_msgs::Image::ConstPtr imageIR3 , const sensor_msgs::Image::ConstPtr imageIR4,
		  const sensor_msgs::Image::ConstPtr imageColor1, const sensor_msgs::Image::ConstPtr imageColor2, const sensor_msgs::Image::ConstPtr imageColor3, const sensor_msgs::Image::ConstPtr imageColor4 )
  {
    cv::Mat ir1, ir2 ,ir3, ir4, color1, color2, color3, color4;


    readImage(imageIR1, ir1);
    readImage(imageIR2, ir2);
    readImage(imageIR3, ir3);
    readImage(imageIR4, ir4);


    readImage(imageColor1, color1);
    readImage(imageColor2, color2);
    readImage(imageColor3, color3);
    readImage(imageColor4, color4);


    lock.lock();
    frame_id1=stripStrTillEnd(imageIR1->header.frame_id, "_");//std::to_string(imageDepth1->header.stamp.toNSec());;
    frame_id2=stripStrTillEnd(imageIR2->header.frame_id, "_");
    frame_id3=stripStrTillEnd(imageIR3->header.frame_id, "_");
    frame_id4=stripStrTillEnd(imageIR4->header.frame_id, "_");

    time_stamp1 = stripStrTillEnd(std::to_string(imageIR1->header.stamp.toNSec()/1000000), "_");
    time_stamp2 = stripStrTillEnd(std::to_string(imageIR2->header.stamp.toNSec()/1000000), "_");
   // std::cout<<"timestamps:" <<time_stamp1<<" "<<time_stamp2<<std::endl;
    this->color1 = color1;
    this->ir1 = ir1;
    this->color2 = color2;
    this->ir2 = ir2;
    this->color3 = color3;
    this->ir3 = ir3;
    this->color4 = color4;
    this->ir4 = ir4;




    updateImage = true;
    lock.unlock();




  }


void callback1(const sensor_msgs::Image::ConstPtr imageIR1, const sensor_msgs::Image::ConstPtr imageIR2 ,const sensor_msgs::Image::ConstPtr imageIR3 , const sensor_msgs::Image::ConstPtr imageIR4,
		  const sensor_msgs::Image::ConstPtr imageIR5, const sensor_msgs::Image::ConstPtr imageIR6)
  {
    cv::Mat ir1, ir2 ,ir3, ir4, ir5, ir6;


    readImage(imageIR1, ir1);
    readImage(imageIR2, ir2);
    readImage(imageIR3, ir3);
    readImage(imageIR4, ir4);
    readImage(imageIR5, ir5);
    readImage(imageIR6, ir6);
    

    
    //lock.lock();
    frame_id1=stripStrTillEnd(imageIR1->header.frame_id, "_");//std::to_string(imageDepth1->header.stamp.toNSec());;
    frame_id2=stripStrTillEnd(imageIR2->header.frame_id, "_");
    frame_id3=stripStrTillEnd(imageIR3->header.frame_id, "_");
    frame_id4=stripStrTillEnd(imageIR4->header.frame_id, "_");
    frame_id5=stripStrTillEnd(imageIR5->header.frame_id, "_");
    frame_id6=stripStrTillEnd(imageIR6->header.frame_id, "_");

    time_stamp1 = stripStrTillEnd(std::to_string(imageIR1->header.stamp.toNSec()/1000000), "_");
    time_stamp2 = stripStrTillEnd(std::to_string(imageIR2->header.stamp.toNSec()/1000000), "_");
   // std::cout<<"timestamps:" <<time_stamp1<<" "<<time_stamp2<<std::endl;
    {
       	boost::mutex::scoped_lock lk(mutex);

        this->ir1 = ir1;
        this->ir2 = ir2;
        this->ir3 = ir3;
        this->ir4 = ir4;
        this->ir5 = ir5;
        this->ir6 = ir6;
        irOk=true;
    }

    data_ready_cond_.notify_one();

   // updateImage1 = true;
    //lock.unlock();




  }

  void callback2( const sensor_msgs::Image::ConstPtr imageColor1, const sensor_msgs::Image::ConstPtr imageColor2, const sensor_msgs::Image::ConstPtr imageColor3, const sensor_msgs::Image::ConstPtr imageColor4 ,  const sensor_msgs::Image::ConstPtr imageColor5, const sensor_msgs::Image::ConstPtr imageColor6)
  {
    cv::Mat color1, color2, color3, color4, color5, color6;;




    readImage(imageColor1, color1);
    readImage(imageColor2, color2);
    readImage(imageColor3, color3);
    readImage(imageColor4, color4);
    readImage(imageColor5, color5);
    readImage(imageColor6, color6);

    //lock.lock();
    frame_color_id1=stripStrTillEnd(imageColor1->header.frame_id, "_");//std::to_string(imageDepth1->header.stamp.toNSec());;
    frame_color_id2=stripStrTillEnd(imageColor2->header.frame_id, "_");
    frame_color_id3=stripStrTillEnd(imageColor3->header.frame_id, "_");
    frame_color_id4=stripStrTillEnd(imageColor4->header.frame_id, "_");
    frame_color_id5=stripStrTillEnd(imageColor5->header.frame_id, "_");
    frame_color_id6=stripStrTillEnd(imageColor6->header.frame_id, "_");

    time_color_stamp1 = stripStrTillEnd(std::to_string(imageColor1->header.stamp.toNSec()/1000000), "_");
    time_color_stamp2 = stripStrTillEnd(std::to_string(imageColor2->header.stamp.toNSec()/1000000), "_");
   // std::cout<<"timestamps:" <<time_stamp1<<" "<<time_stamp2<<std::endl;
   {
       	boost::mutex::scoped_lock lk(mutex);

      
        this->color1 = color1;
        this->color2 = color2;
        this->color3 = color3;
        this->color4 = color4;
        this->color5 = color5;
        this->color6 = color6;
        colorOk=true;
   }


   // updateImage2 = true;
   // lock.unlock();
  data_ready_cond_.notify_one();



  }


//
static void create_xy_table(const cv::Mat  &xy_table, const std::string &filename)
{

    int width = xy_table.rows;
    int height = xy_table.cols;
    struct float2 {

      float x,y;
    } ;
   
   float2* table_data=new float2[width*height];

     std::ofstream file;
  
   file.open(filename, std::ios::binary);
    for (int y = 0, idx = 0; y < height; y++)
    {
  
        for (int x = 0; x < width; x++, idx++)
        {
          table_data[idx].x = xy_table.at<cv::Vec2f>(y, x)[0];
          table_data[idx].y = xy_table.at<cv::Vec2f>(y, x)[1];
          


            file.write((char *)&table_data[idx], sizeof(table_data[idx]));
          //  std::cout<<"size of table "<<sizeof(table_data)<<std::endl;
        }

    }
    file.close();
}


static void createCloud(const cv::Mat &depth, const cv::Mat & xy_table, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
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
   







  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }


  


  



  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
	cameraMatrix  = cv::Mat::zeros(4, 3, CV_64F);
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
    
    *itC = cameraInfo->height;
    ++itC;
    *itC = cameraInfo->width;
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

 //   std::cout<<cameraMatrix<<std::endl;
    return cameraMatrix;
  }


 void createTopicDepthandImagesCamInfosTuple(const std::vector<std::string>  &serials) {


  	// for(auto & s: serials ) {
  	//   topicsIRAndColorCamInfos_.push_back(std::make_tuple("/k4A_" + s + "/hd/image_depth", "/k4A_" + s + "/hd/image_ir",
  	// 		  "/k4A_" + s + "/hd/depth_camera_info", "/k4A_" + s + "/hd/ir_camera_info"));

  	// }

	for(auto & s: serials ) {
  	  		topicsIRAndColorCamInfos_.push_back(std::make_tuple("/k4A_" + s + "/hd/image_depth",
					       					"/k4A_" + s + "/hd/image_ir", 
										"/k4A_" + s + "/hd/image_color",
  			  							"/k4A_" + s + "/hd/depth_camera_info", 
										"/k4A_" + s + "/hd/ir_camera_info", 
										"/k4A_" + s + "/hd/color_camera_info", 
										"/k4A_" + s + "/hd/xy_table"));

  	}


  }

std::vector<std::string> createListOfSerials() {
        std::vector<std::string> strVec;
        strVec=serials_;
        for (int i = 0 ; i < serials_.size() - 1; i++) {

            std::string tmp=serials_[i]+"_"+serials_[i+1];
            strVec.push_back(tmp);

        }
         strVec.push_back(serials_[0]+"_"+serials_[serials_.size()-1]);

        return strVec;

    }

int remove_directory(const char *path) {
   DIR *d = opendir(path);
   size_t path_len = strlen(path);
   int r = -1;

   if (d) {
      struct dirent *p;

      r = 0;
      while (!r && (p=readdir(d))) {
          int r2 = -1;
          char *buf;
          size_t len;

          /* Skip the names "." and ".." as we don't want to recurse on them. */
          if (!strcmp(p->d_name, ".") || !strcmp(p->d_name, ".."))
             continue;

          len = path_len + strlen(p->d_name) + 2; 
          buf = (char*)malloc(len);

          if (buf) {
             struct stat statbuf;

             snprintf(buf, len, "%s/%s", path, p->d_name);
             if (!stat(buf, &statbuf)) {
                if (S_ISDIR(statbuf.st_mode))
                   r2 = remove_directory(buf);
                else
                   r2 = unlink(buf);
             }
             free(buf);
          }
          r = r2;
      }
      closedir(d);
   }

   if (!r)
      r = rmdir(path);

   return r;
}

void list_dir (const char *dirname) {
 DIR *d;
  struct dirent *dir;
  d = opendir(dirname);
  if (d) {
    while ((dir = readdir(d)) != NULL) {
      printf("%s\n", dir->d_name);
    }
    closedir(d);
  }

}
 void imageViewer()
  {
    cv::Mat color1, ir1, color2, ir2, color3, ir3, color4, ir4, color5, ir5, color6, ir6;
    cv::namedWindow("Color Image 1");
    cv::moveWindow("Color Image 1", 250,0);
    cv::namedWindow("Color Image 2");
    cv::moveWindow("Color Image 2", 500,0);

     cv::namedWindow("Color Image 3");
    cv::moveWindow("Color Image 3", 800,0);

    cv::namedWindow("Color Image 4");
    cv::moveWindow("Color Image 4", 1000,0);



   // cv::namedWindow("Image Viewer");
   // oss << "starting...";
    int file_cnt=0;
    std::vector<std::string> serials=createListOfSerials();

    for(int i = 0; running && ros::ok();)
    {
//       if(updateImage1 && updateImage2)
//       {
//         lock.lock();
//         color1 = this->color1;
//         ir1 = this->ir1;
//         color2 = this->color2;
//         ir2 = this->ir2;
//         color3 = this->color3;
//         ir3 = this->ir3;
// 	    color4 = this->color4;
//         ir4 = this->ir4;
// 	    color5 = this->color5;
//         ir5 = this->ir5;
// 	    color6 = this->color6;
//         ir6 = this->ir6;

//         updateImage1 = false; updateImage2=false;
//         lock.unlock();



//         cv::imshow("Color Image 1", color1);
//       //  cv::moveWindow("Color Image 1", 250,0);
//         cv::imshow("Color Image 2", color2);
//       //  cv::moveWindow("Color Image 2", 800,0);
//         cv::imshow("Color Image 3", color3);
//         cv::imshow("Color Image 4", color4);
// //        cv::Mat matDst(cv::Size(color1.cols*2,color1.rows),color1.type(),cv::Scalar::all(0));
// //        cv::Mat matRoi = matDst(cv::Rect(0,0,color1.cols,color1.rows));
// //        color1.copyTo(matRoi);
// //        matRoi = matDst(cv::Rect(color1.cols,0,color1.cols,color1.rows));
// //        color2.copyTo(matRoi);
// //
// //        imshow("",color1);

//       }

       {
						boost::mutex::scoped_lock lk(mutex);

            data_ready_cond_.wait(lk, [this] { return ( irOk && colorOk  ); });
            //std::cout<<"Inside imageviewer 1\n";
            irOk=false; colorOk=false;

            color1 = this->color1;
            ir1 = this->ir1;
            color2 = this->color2;
            ir2 = this->ir2;
            color3 = this->color3;
            ir3 = this->ir3;
            color4 = this->color4;
            ir4 = this->ir4;
            color5 = this->color5;
            ir5 = this->ir5;
            color6 = this->color6;
            ir6 = this->ir6;

            cv::imshow("Color Image 1", color1);
            //  cv::moveWindow("Color Image 1", 250,0);
   //         cv::imshow("Color Image 2", color2);
            //  cv::moveWindow("Color Image 2", 800,0);
     //       cv::imshow("Color Image 3", color3);
       //     cv::imshow("Color Image 4", color4);

       }

      int key = cv::waitKey(10);
      switch(key & 0xFF)
      {
        case 27:
        case 'q':
            running = false;
            break;

        case 'c':

            if (i < serials.size()) {
               // struct stat sb;

               // if (stat(serials[i].c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
                //    std::cout<<"Already exists" <<std::endl;
               // } else {
                    mkdir(serials[i].c_str(), 0700);
                    chdir(serials[i].c_str());
                    char tmp[256];
                    getcwd(tmp, sizeof(tmp));
                    std::cout<<"Current working directory:"<<tmp<<std::endl;
                    i++; 


               // }


            } else std::cout<<"No more serial number!!! Did not create directory"<<std::endl;
            break;
        case 'l':
            char tmp[256];
             getcwd(tmp, sizeof(tmp));
             list_dir(tmp);
             break;
        case 'r':
            //char tmp[256];
            getcwd(tmp, sizeof(tmp));
            std::cout<<"Delete "<<tmp<< " yes/no ?"<<std::endl;
            char temp;
            std::cin >> temp;
            if (temp=='y') {
                remove_directory(tmp);
                std::cout<<"removed: "<<tmp<<std::endl;
                chdir("../");
                i--;
            } else if (temp == 'n') {

            
                
            }
            getcwd(tmp, sizeof(tmp));
            std::cout<<"Current working directory:"<<tmp<<std::endl;
            
            break;
        case 'p':
            {
                char tmp[256];   
                getcwd(tmp, sizeof(tmp));
                std::cout<<"Current working directory:"<< tmp<<std::endl;
            }
            break;
        case 'u':

            chdir("../");
           
            getcwd(tmp, sizeof(tmp));
            std::cout<<"Current working directory:"<< tmp<<std::endl;
            break;


      case ' ':
      case 's':
    	  std::cout<<"saving colors  \n";
    	//  std::cout<<"size of dpth : "<<color1.elemSize()<<" "<< color1.elemSize1()<<std::endl;
    //	 cv::imwrite(frame_id1 + "_" + std::to_string(file_cnt)+ "_0.png", color1, params);
    //	cv::imwrite(frame_id2 + "_" + std::to_string(file_cnt)+ "_1.png", color2, params);
        std::cout<<"stamps "<<time_stamp1<<" "<<time_color_stamp1<<std::endl;
	      std::string tmstp=time_stamp1;
	      cv::imwrite(frame_color_id1 + "-" + tmstp + "-rgb.png", color1, params);
        cv::imwrite(frame_color_id2 + "-" + tmstp + "-rgb.png", color2, params);
        cv::imwrite(frame_color_id3 + "-" + tmstp + "-rgb.png", color3, params);
	      cv::imwrite(frame_color_id4 + "-" + tmstp + "-rgb.png", color4, params);
        cv::imwrite(frame_color_id5 + "-" + tmstp + "-rgb.png", color5, params);
        cv::imwrite(frame_color_id6 + "-" + tmstp + "-rgb.png", color6, params);

//        writer.write(color1);
//        writer.write(color2);
//        writer.write(color3);
//        writer.write(color4);
//        writer.write(color5);
//        writer.write(color6);
//        
        /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());

        cloud1->height = depth1.rows;
        cloud1->width = depth1.cols;
        cloud1->is_dense = false;
        cloud1->points.resize(cloud1->height * cloud1->width); 

        createCloud(depth1, mapSerialDepthXYTables[serials_[0]], cloud1);
        plyWriter.write(serials_[0] +".ply", *cloud1, true, false);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>()); 

        cloud2->height = depth2.rows;
        cloud2->width = depth2.cols;
        cloud2->is_dense = false;
        cloud2->points.resize(cloud2->height * cloud2->width); 

        createCloud(depth2, mapSerialDepthXYTables[serials_[1]], cloud2);
        plyWriter.write(serials_[1] +".ply", *cloud2, true, false);
*/
        //for (auto &s:serials_) {
         // cv::FileStorage storage(s+"_ir_calib.xml", cv::FileStorage::WRITE);
         // storage <<"caminfo"<<mapSerialColorCamInfo[s];
         // storage.release();
 //         cv::FileStorage storage1(s+"_depth_calib.xml", cv::FileStorage::WRITE);
   //       storage1 <<"caminfo"<< mapSerialColorCamInfo[s];
     //     storage1.release();

        //  create_xy_table(mapSerialDepthXYTables[s], s+"_tablexy.data");
    //std::cout<<" mapSerialColorCamInfo[s]); " <<  mapSerialColorCamInfo[s]<<std::endl;
    //cv::imwrite(s+"_color_calib.xml" ,  mapSerialColorCamInfo[s]);
            //cv::imwrite(s+"_depth_calib.xml" ,   mapSerialDepthCamInfo[s]);
      //  }
          
          
    	    std::cout<<"saving IRs \n";

    	 // std::cout<<"size of dpth : "<<depth1.elemSize()<<" "<< depth1.elemSize1()<<std::endl;
    //	  cv::imwrite(frame_id1 + "_" + std::to_string(file_cnt) + "_0_depth.png", depth1, params);
    //	  cv::imwrite(frame_id2 + "_" + std::to_string(file_cnt) + "_1_depth.png", depth2, params);
    	    cv::imwrite(frame_id1 + "-" + tmstp + "-ir.png", ir1, params);
          cv::imwrite(frame_id2 + "-" + tmstp + "-ir.png", ir2, params);
          cv::imwrite(frame_id3 + "-" + tmstp + "-ir.png", ir3, params);
          cv::imwrite(frame_id4 + "-" + tmstp + "-ir.png", ir4, params);
          cv::imwrite(frame_id5 + "-" + tmstp + "-ir.png", ir5, params);
          cv::imwrite(frame_id6 + "-" + tmstp + "-ir.png", ir6, params);

	        file_cnt++;
          break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }





};


int main(int argc, char **argv)
{

	ros::init(argc, argv, "kinect2_viewer");

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
	 //   std::cout<<"serial "<<serial<<std::endl;

	}


	if(!ros::ok())
	{
	return 0;
	}

	bool useExact=false;
	bool useCompressed=true;


	Receiver receiver(serials , useExact, useCompressed);

	receiver.start();

	receiver.stop();



	ros::shutdown();
	return 0;
}
