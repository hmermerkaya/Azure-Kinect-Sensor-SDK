#include <chrono>
//#include <pcl/pcl_base.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "k4a/k4a.h"
#include <ros/ros.h>
#include <fstream>
class k4AWithROS {
    public:
    k4AWithROS(int deviceIndex,std::string serial,  k4a_device_configuration_t config, k4a_device_t device):device_(device), config_(config) {

		k4a_device_open(deviceIndex, &device_);
	
		k4a_device_start_cameras(device_, &config_);



		
		if (K4A_RESULT_SUCCEEDED !=
			k4a_device_get_calibration(device_, config_.depth_mode, config_.color_resolution, &calibration_))
		{
			printf("Failed to get calibration\n");
			exit(1) ;
		}

		
        header_ir_.frame_id = "k4A_ir_optical_frame_"+serial;
		header_depth_.frame_id = "k4A_depth_optical_frame_"+serial;
		header_color_.frame_id = "k4A_color_optical_frame_"+serial;
        header_xy_table_.frame_id="k4A_depth_optical_frame_"+serial;

        depth_pubCompressed_ = nh_.advertise<sensor_msgs::CompressedImage>("/k4A_" + serial + "/hd/image_depth/compressed", 10);
		ir_pubCompressed_ = nh_.advertise<sensor_msgs::CompressedImage>("/k4A_" + serial + "/hd/image_ir/compressed", 10);
		color_pubCompressed_= nh_.advertise<sensor_msgs::CompressedImage>("/k4A_" + serial + "/hd/image_color/compressed", 10);
		transformed_color_pubCompressed_= nh_.advertise<sensor_msgs::CompressedImage>("/k4A_" + serial + "/hd/transformed_image_color/compressed", 10);

		depth_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/k4A_" + serial + "/hd/depth_camera_info", 10);
		ir_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/k4A_" + serial + "/hd/ir_camera_info", 10);
		color_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/k4A_" + serial + "/hd/color_camera_info", 10);

        xy_table_pub_ =nh_.advertise<sensor_msgs::Image>("/k4A_" + serial + "/hd/xy_table",10); 

		size_depth_=cv::Size(calibration_.depth_camera_calibration.resolution_width, calibration_.depth_camera_calibration.resolution_height);
		size_color_=cv::Size(calibration_.color_camera_calibration.resolution_width, calibration_.color_camera_calibration.resolution_height);
		size_ir_=cv::Size(calibration_.depth_camera_calibration.resolution_width, calibration_.depth_camera_calibration.resolution_height);
		//std::cout<<size_depth_.width<< " "<<size_depth_.height<<std::endl;
		createCameraInfoIR(calibration_);
		createCameraInfoDepth(calibration_);
		createCameraInfoColor(calibration_);
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     calibration_.depth_camera_calibration.resolution_width,
                      calibration_.depth_camera_calibration.resolution_height,
                      calibration_.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                      &xy_table);
		create_xy_table(&calibration_, xy_table);
        xy_table_mat_=cv::Mat(calibration_.depth_camera_calibration.resolution_height ,
            calibration_.depth_camera_calibration.resolution_width, CV_32FC2, (void *)k4a_image_get_buffer(xy_table));
        header_xy_table_.stamp=ros::Time().now();
        createImage(xy_table_mat_, header_xy_table_, xy_table_image, false);


    }
 

template<typename T>
inline void ConvertToGrayScaleImage(const T* imgDat, const int size, const int vmin, const int vmax, uint8_t* img)
{
    for (int i = 0; i < size; i++)
    {
        T v = imgDat[i];
        float colorValue = 0.0f;
        if (v <= vmin)
        {
            colorValue = 0.0f;
        }
        else if (v >= vmax)
        {
            colorValue = 1.0f;
        }
        else
        {
            colorValue = (float)(v - vmin) / (float)(vmax - vmin);
        }
        img[i] = (uint8_t)(colorValue * 255);
	}
}
void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
    k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}

void publishDepthColorCompressedAndCameraInfos(const ros::Time & tm = ros::Time::now(), const bool full_hd = true) {

        initCompression(jpeg_quality, png_level, use_png);

      //  k2g_.get(tmp_color, tmp_depth, full_hd, remove_points);
	//	std::cout<<"step 1"<<std::endl;
		k4a_capture_t capture;
		
		 switch (k4a_device_get_capture(device_, &capture, TIMEOUT_IN_MS))
		{
		case K4A_WAIT_RESULT_SUCCEEDED:
			break;
		case K4A_WAIT_RESULT_TIMEOUT:
			printf("Timed out waiting for a capture\n");
			return;
		case K4A_WAIT_RESULT_FAILED:
			printf("Failed to read a capture\n");
			return;
		}
		
		//k4a_device_get_capture(device_, &capture, TIMEOUT_IN_MS);
       // k4a_image_t ir_image = k4a_capture_get_ir_image(capture);
        k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
		if (depth_image == 0)
		{
			printf("Failed to get depth image from capture\n");
			return;
		}


        k4a_image_t ir_image = k4a_capture_get_ir_image(capture);
		if (ir_image == 0)
		{
			printf("Failed to get color image from capture\n");
			return;
		}
 		k4a_image_t color_image = k4a_capture_get_color_image(capture);
		if (color_image == 0)
		{
			printf("Failed to get color image from capture\n");
			return;
		}
		// k4a_image_t xy_table = NULL;
		// k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        //              calibration_.depth_camera_calibration.resolution_width,
        //              calibration_.depth_camera_calibration.resolution_height,
        //              calibration_.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
        //              &xy_table);
		// create_xy_table(&calibration_, xy_table);
		//std::cout<<"step 2"<<std::endl;
		//depth image
		int height = k4a_image_get_height_pixels(depth_image);
        int width = k4a_image_get_width_pixels(depth_image);
		//std::cout<<"step 3"<<std::endl;
		cv::Mat tmp_depth(height , width, CV_16UC1, (void *)k4a_image_get_buffer(depth_image));

		k4a_image_t transformed_color_image = NULL;
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
													width,
													height,
													width * (int)sizeof(uint16_t),
													&transformed_color_image))
		{
			printf("Failed to create transformed color image\n");
			return ;
		}
		k4a_image_t point_cloud_image = NULL;
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
													width,
													height,
													width * 3 * (int)sizeof(int16_t),
													&point_cloud_image))
		{
			printf("Failed to create point cloud image\n");
			return ;
		}

		k4a_transformation_t transformation = NULL;
		transformation = k4a_transformation_create(&calibration_);
		
		if (K4A_RESULT_SUCCEEDED !=
        	k4a_transformation_color_image_to_depth_camera(transformation, depth_image, color_image, transformed_color_image))
    	{
        	printf("Failed to compute transformed color image\n");
        	return ;
    	}

		if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation,
                                                                              depth_image,
                                                                              K4A_CALIBRATION_TYPE_DEPTH,
                                                                              point_cloud_image))
		{
			printf("Failed to compute point cloud\n");
			return ;
		}

		cv::Mat tmp_transformed_color(height , width, CV_8UC4, (void *)k4a_image_get_buffer(transformed_color_image));




		//cv::Mat tmp_xytable(height , width, CV_32FC2, (void *)k4a_image_get_buffer(xy_table));
		//color image
		//std::cout<<"step 4"<<std::endl;
		height = k4a_image_get_height_pixels(color_image);
        width = k4a_image_get_width_pixels(color_image);
		cv::Mat tmp_color(height , width, CV_8UC4, (void *)k4a_image_get_buffer(color_image));
		
		



		height = k4a_image_get_height_pixels(ir_image);
        width = k4a_image_get_width_pixels(ir_image);
		
		uint8_t* imgData = k4a_image_get_buffer(ir_image);
		uint16_t* irImg = reinterpret_cast<uint16_t*>(imgData);
		std::vector<uint8_t> grayScaleImg(width * height);
		int irMinValue = 0;
		int irMaxValue = 1000;
		ConvertToGrayScaleImage(irImg, width * height, irMinValue, irMaxValue, grayScaleImg.data());
		const cv::Mat tmp_ir(cv::Size(width, height), CV_8U, grayScaleImg.data());


		//std::cout<<"step 5"<<std::endl;
	//	cv::Mat tmp_ir(height2 , width2, CV_8U, (void *)k4a_image_get_buffer(ir_image));
	//	 std::vector<int> compression_params;
    //	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //	compression_params.push_back(9);
	//	cv::imwrite("sample.png", tmp_color,compression_params);
	//	std::cout<<"step 6"<<std::endl;

                //std::cout<<"tmp_depth "<<tmp_depth.rows<<std::endl;
        header_depth_.stamp = tm; //ros::Time::now();
        createCompressed(tmp_depth, header_depth_, DEPTH_HD, depth_imageCompressed);
		depth_pubCompressed_.publish(depth_imageCompressed);
		//std::cout<<"step 7"<<std::endl;
                //  std::cout<<"after tmp_depth "<<tmp_depth.rows<<std::endl;
        header_color_.stamp = tm; //ros::Time::now();
        createCompressed(tmp_color, header_color_, COLOR_HD, color_imageCompressed);
		color_pubCompressed_.publish(color_imageCompressed);

		header_color_.stamp = tm; //ros::Time::now();
        createCompressed(tmp_transformed_color, header_color_, COLOR_HD, transformed_color_imageCompressed);
		transformed_color_pubCompressed_.publish(transformed_color_imageCompressed);

        
		//std::cout<<"step 8"<<std::endl;
        header_ir_.stamp = tm; //ros::Time::now();
        createCompressed(tmp_ir, header_ir_, MONO_HD, ir_imageCompressed);
		//std::cout<<"step 9"<<std::endl;
        ir_pubCompressed_.publish(ir_imageCompressed);

        camera_info_depth_.header=header_depth_;
        camera_info_ir_.header=header_ir_;
		camera_info_color_.header=header_color_;

        depth_info_pub_.publish(camera_info_depth_);
        ir_info_pub_.publish(camera_info_ir_);
		color_info_pub_.publish(camera_info_color_);
        xy_table_pub_.publish(xy_table_image);

		//std::cout<<"step 12"<<std::endl;

 		k4a_image_release(ir_image);
        k4a_image_release(depth_image);
		k4a_image_release(color_image);
		k4a_image_release(transformed_color_image);
	//	k4a_image_release(xy_table);
        k4a_capture_release(capture);

//		header_color_.stamp = tm;//ros::Time::now();
//		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
//		color_pub_.publish(color_image_);

	}
	void shutdown(){
    k4a_image_release(xy_table); 
	k4a_device_close(device_);

	}

	bool terminate () {

		return stop;
	}

	void setShutdown(){
		stop=true;
	}
private:
	  enum Image
	  {
	    IR_SD = 0,
	    IR_SD_RECT,

	    DEPTH_SD,
	    DEPTH_SD_RECT,
	    DEPTH_HD,
	    DEPTH_QHD,

	    COLOR_SD_RECT,
	    COLOR_HD,
	    COLOR_HD_RECT,
	    COLOR_QHD,
	    COLOR_QHD_RECT,

	    MONO_HD,
	    MONO_HD_RECT,
	    MONO_QHD,
	    MONO_QHD_RECT,

	    COUNT
	  };


	void initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png)
		  {
			compressionParams.resize(7, 0);
			compressionParams[0] = cv::IMWRITE_JPEG_QUALITY;
			compressionParams[1] = jpegQuality;
			compressionParams[2] = cv::IMWRITE_PNG_COMPRESSION;
			compressionParams[3] = pngLevel;
			compressionParams[4] = cv::IMWRITE_PNG_STRATEGY;
			compressionParams[5] = cv::IMWRITE_PNG_STRATEGY_RLE;
			compressionParams[6] = 0;

			if(use_png)
			{
			  compression16BitExt = ".png";
			  compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; png compressed";
			}
			else
			{
			  compression16BitExt = ".tif";
			  compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; tiff compressed";
			}
		}

	void createImage(cv::Mat & image, std_msgs::Header & header, sensor_msgs::Image & msgImage, const bool color) const
	{	
		size_t step, size;
		step = image.cols * image.elemSize();
		size = image.rows * step;
		if(color)
			msgImage.encoding = sensor_msgs::image_encodings::BGRA8;
		else
			msgImage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;

		msgImage.header = header;
        msgImage.height = image.rows;
		msgImage.width = image.cols;
		msgImage.is_bigendian = false;
		msgImage.step = step;
		msgImage.data.resize(size);

		memcpy(msgImage.data.data(), image.data, size);
	}


	void createCompressed(const cv::Mat &image, const std_msgs::Header &header, const Image type, sensor_msgs::CompressedImage &msgImage) const
	  {
	    msgImage.header = header;

	    switch(type)
	    {
	    case IR_SD:
	    case IR_SD_RECT:
	    case DEPTH_SD:
	    case DEPTH_SD_RECT:
	    case DEPTH_HD:
	    case DEPTH_QHD:
	      msgImage.format = compression16BitString;
	      cv::imencode(compression16BitExt, image, msgImage.data, compressionParams);
	      break;
	    case COLOR_SD_RECT:
	    case COLOR_HD:
	    case COLOR_HD_RECT:
	    case COLOR_QHD:
	    case COLOR_QHD_RECT:
	       msgImage.format = sensor_msgs::image_encodings::BGR8 + "; jpeg compressed bgr8";
		//	msgImage.format = sensor_msgs::image_encodings::TYPE_8UC4+ "; png compressed";
			//std::cout<<"step 2.1"<<std::endl;
	      	cv::imencode(".jpg", image, msgImage.data, compressionParams);
			 //std::cout<<"step 2.2"<<std::endl; 
	      	break;
	    case MONO_HD:
	    case MONO_HD_RECT:
	    case MONO_QHD:
	    case MONO_QHD_RECT:
	      msgImage.format = sensor_msgs::image_encodings::TYPE_8UC1 + "; jpeg compressed ";
	      cv::imencode(".jpg", image, msgImage.data, compressionParams);
	      break;
	    case COUNT:
	      return;
	    }
	  }


    

    void createCameraInfoDepth(k4a_calibration_t depth_params)
        {
        cv::Mat proj_matrix_depth = cv::Mat::zeros(3, 4, CV_64F);
        cv::Mat camera_matrix_depth = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat distortion_matrix_depth = cv::Mat::zeros(1, 8, CV_64F);

		distortion_matrix_depth.at<double>(0,1)=depth_params.depth_camera_calibration.intrinsics.parameters.param.k1;
		distortion_matrix_depth.at<double>(0,2)=depth_params.depth_camera_calibration.intrinsics.parameters.param.k2;
		distortion_matrix_depth.at<double>(0,3)=depth_params.depth_camera_calibration.intrinsics.parameters.param.k3;
		distortion_matrix_depth.at<double>(0,4)=depth_params.depth_camera_calibration.intrinsics.parameters.param.k4;
		distortion_matrix_depth.at<double>(0,5)=depth_params.depth_camera_calibration.intrinsics.parameters.param.k5;
		distortion_matrix_depth.at<double>(0,6)=depth_params.depth_camera_calibration.intrinsics.parameters.param.k6;
		distortion_matrix_depth.at<double>(0,7)=depth_params.depth_camera_calibration.intrinsics.parameters.param.p1;
		distortion_matrix_depth.at<double>(0,8)=depth_params.depth_camera_calibration.intrinsics.parameters.param.p2;

        camera_matrix_depth.at<double>(0, 0) = depth_params.depth_camera_calibration.intrinsics.parameters.param.fx;
        camera_matrix_depth.at<double>(1, 1) = depth_params.depth_camera_calibration.intrinsics.parameters.param.fy;
        camera_matrix_depth.at<double>(0, 2) = depth_params.depth_camera_calibration.intrinsics.parameters.param.cx;
        camera_matrix_depth.at<double>(1, 2) = depth_params.depth_camera_calibration.intrinsics.parameters.param.cy;
        camera_matrix_depth.at<double>(2, 2) = 1;
        camera_matrix_depth.copyTo(proj_matrix_depth(cv::Rect(0, 0, 3, 3)));
        
        createCameraInfo(size_depth_, camera_matrix_depth, distortion_matrix_depth, cv::Mat::eye(3, 3, CV_64F), 
                                                                        proj_matrix_depth, camera_info_depth_, true);
        }

	void createCameraInfoIR(k4a_calibration_t ir_params)
	{
		cv::Mat proj_matrix_ir = cv::Mat::zeros(3, 4, CV_64F);
		cv::Mat camera_matrix_ir = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat distortion_matrix_ir = cv::Mat::zeros(1, 5, CV_64F);

		camera_matrix_ir.at<double>(0, 0) = ir_params.depth_camera_calibration.intrinsics.parameters.param.fx;
		camera_matrix_ir.at<double>(1, 1) = ir_params.depth_camera_calibration.intrinsics.parameters.param.fy;
		camera_matrix_ir.at<double>(0, 2) = ir_params.depth_camera_calibration.intrinsics.parameters.param.cx;
		camera_matrix_ir.at<double>(1, 2) = ir_params.depth_camera_calibration.intrinsics.parameters.param.cy;
		camera_matrix_ir.at<double>(2, 2) = 1;
		camera_matrix_ir.copyTo(proj_matrix_ir(cv::Rect(0, 0, 3, 3)));
		
		createCameraInfo(size_ir_, camera_matrix_ir, distortion_matrix_ir, cv::Mat::eye(3, 3, CV_64F), 
																		   proj_matrix_ir, camera_info_ir_, false);
	}
	void createCameraInfoColor(k4a_calibration_t color_params)
	{
		cv::Mat proj_matrix_color = cv::Mat::zeros(3, 4, CV_64F);
		cv::Mat camera_matrix_color = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat distortion_matrix_color = cv::Mat::zeros(1, 5, CV_64F);

		camera_matrix_color.at<double>(0, 0) = color_params.depth_camera_calibration.intrinsics.parameters.param.fx;
		camera_matrix_color.at<double>(1, 1) = color_params.depth_camera_calibration.intrinsics.parameters.param.fy;
		camera_matrix_color.at<double>(0, 2) = color_params.depth_camera_calibration.intrinsics.parameters.param.cx;
		camera_matrix_color.at<double>(1, 2) = color_params.depth_camera_calibration.intrinsics.parameters.param.cy;
		camera_matrix_color.at<double>(2, 2) = 1;
		camera_matrix_color.copyTo(proj_matrix_color(cv::Rect(0, 0, 3, 3)));
		
		createCameraInfo(size_depth_, camera_matrix_color, distortion_matrix_color, cv::Mat::eye(3, 3, CV_64F), 
																		   proj_matrix_color, camera_info_color_, true);
	}


	void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, 
						  const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo, const bool color ) const
	{

		if (color)
		{
			cameraInfo.header.frame_id = "k4A_rgb_optical_frame";	
		}
		else
		{
			cameraInfo.header.frame_id = "k4A_ir_optical_frame";	
		}
		cameraInfo.height = size.height;
		cameraInfo.width = size.width;

		const double *itC = cameraMatrix.ptr<double>(0, 0);
		for(size_t i = 0; i < 9; ++i, ++itC)
		{
			cameraInfo.K[i] = *itC;
		}

		const double *itR = rotation.ptr<double>(0, 0);
		for(size_t i = 0; i < 9; ++i, ++itR)
		{
			cameraInfo.R[i] = *itR;
		}

		const double *itP = projection.ptr<double>(0, 0);
		for(size_t i = 0; i < 12; ++i, ++itP)
		{
			cameraInfo.P[i] = *itP;
		}

		cameraInfo.distortion_model = "plumb_bob";
		cameraInfo.D.resize(distortion.cols);
		const double *itD = distortion.ptr<double>(0, 0);
		
		for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
		{
			cameraInfo.D[i] = *itD;
		}
	}
    ros::NodeHandle nh_;
	ros::Publisher  ir_pubCompressed_, ir_info_pub_, depth_pubCompressed_, 
	depth_info_pub_, color_pubCompressed_, color_info_pub_, transformed_color_pubCompressed_,
	xy_table_pub_;
    
	k4a_image_t xy_table = NULL;
	k4a_calibration_t calibration_;
	cv::Mat xy_table_mat_;
	int32_t TIMEOUT_IN_MS = 1349400;
	k4a_device_t device_;
	k4a_device_configuration_t config_;
    cv::Mat ir_, depth_;
    cv::Size size_ir_, size_depth_, size_color_;
    std_msgs::Header header_depth_, header_ir_, header_color_, header_xy_table_;
    
    sensor_msgs::CompressedImage depth_imageCompressed;
    sensor_msgs::CompressedImage ir_imageCompressed;
    sensor_msgs::CompressedImage color_imageCompressed;
	sensor_msgs::CompressedImage transformed_color_imageCompressed;


	sensor_msgs::Image xy_table_image;

    int32_t jpeg_quality= 80, png_level=1;
    bool use_png=false;

    sensor_msgs::CameraInfo  camera_info_ir_, camera_info_depth_, camera_info_color_;
    std::vector<int> compressionParams;
    std::string compression16BitExt, compression16BitString, baseNameTF;
	bool stop=false;
};
