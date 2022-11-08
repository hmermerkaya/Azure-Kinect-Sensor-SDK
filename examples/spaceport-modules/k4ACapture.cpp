
#include "k4a/k4a.h"
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sensor_msgs/PointCloud2.h>

using namespace cv;

static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table, const std::string &serial_number)
{
    k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;
     std::ofstream file;
   std::string filename(std::string(serial_number)+"_tablexy.data");
   file.open(filename, std::ios::binary);
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
                table_data[idx].xy.x = ray.xyz.x; // std::cout<<"x value "<<(x- calibration->depth_camera_calibration.intrinsics.parameters.param.cx)
                                                     //calibration->depth_camera_calibration.intrinsics.parameters.param.fx<<" "<<ray.xyz.x<<" "<<ray.xyz.z<<std::endl;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }


            file.write((char *)&table_data[idx], sizeof(table_data[idx]));
          //  std::cout<<"size of table "<<sizeof(table_data)<<std::endl;
        }
    }
    file.close();
}
static void  createTransformedColorImageToFile(cv::Mat &transformedColorImg, k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image
                                    )
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    std::cout<<"depth_image_width height "<<depth_image_width_pixels<< " "<<depth_image_height_pixels<<std::endl;
    k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return ;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                                depth_image,
                                                                                color_image,
                                                                                transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return ;
    }

    


    int rows = k4a_image_get_height_pixels(transformed_color_image);
    int cols = k4a_image_get_width_pixels(transformed_color_image);
    cv::Mat tmp(rows , cols, CV_8UC4, (void *)k4a_image_get_buffer(transformed_color_image), cv::Mat::AUTO_STEP);
    transformedColorImg=tmp.clone();
  //  cv::Mat gray_image;
  //  cvtColor( colorMat, gray_image, CV_BGRA2GRAY );
  //  cv::imwrite( "./transformed.png", colorMat );
   // std::cout << "[Streaming Service] Transformed Color frame is stored in " << file_name << std::endl;
  //  return true;
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

int main() {
    k4a_device_t device = NULL;
    k4a_transformation_t transformation = NULL;
    k4a_capture_t capture;
    int32_t TIMEOUT_IN_MS = 1349400;
    uint32_t device_count = k4a_device_get_installed_count();
    printf("Found %d connected devices:\n", device_count);
    k4a_image_t xy_table = NULL;

    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++) {
    
        
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
        {
            printf("%d: Failed to open device\n", deviceIndex);
        }

    //    k4a_device_open(K4A_DEVICE_DEFAULT, &device);
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_15;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;

        k4a_device_start_cameras(device, &config);
        bool running =true;
        cv::namedWindow("foobar");


        char *serial_number = NULL;
        size_t serial_number_length = 0;

        if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(device, NULL, &serial_number_length))
        {
        //   printf("%d: Failed to get serial number length\n", deviceIndex);
            k4a_device_close(device);
            device = NULL;
            return 1;
        }

        serial_number =(char *) malloc(serial_number_length);
        if (serial_number == NULL)
        {
        //  printf("%d: Failed to allocate memory for serial number (%zu bytes)\n", deviceIndex, serial_number_length);
            k4a_device_close(device);
            device = NULL;
            return 1;
        }

        if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(device, serial_number, &serial_number_length))
        {
        //  printf("%d: Failed to get serial number\n", deviceIndex);
            free(serial_number);
            serial_number = NULL;
            k4a_device_close(device);
            device = NULL;
            return 1;
        }



        k4a_calibration_t calibration;
        if (K4A_RESULT_SUCCEEDED !=
            k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
        {
            printf("Failed to get calibration\n");
            return 1;
        }

        transformation = k4a_transformation_create(&calibration);

        auto calib = calibration.depth_camera_calibration;
        struct _reducedCalib{
        int w, h;
        float cx, cy, fx, fy, k1, k2, k3, k4, k5, k6;

        } ;
        
        _reducedCalib reducedCalib={calib.resolution_width, calib.resolution_height, calib.intrinsics.parameters.param.cx,
        calib.intrinsics.parameters.param.cy, calib.intrinsics.parameters.param.fx , calib.intrinsics.parameters.param.fy,
        calib.intrinsics.parameters.param.k1, calib.intrinsics.parameters.param.k2, calib.intrinsics.parameters.param.k3,
        calib.intrinsics.parameters.param.k4, calib.intrinsics.parameters.param.k5, calib.intrinsics.parameters.param.k6
        };
        
        std::cout<<"calibs "<<calib.intrinsics.parameters.param.cx<<" "<<calib.intrinsics.parameters.param.cy<<std::endl;
        std::ofstream file;
        std::string filename(std::string(serial_number)+"_calib.data");
        file.open(filename, std::ios::binary);
        file.write((char *)&reducedCalib, sizeof(reducedCalib));

         k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     calibration.depth_camera_calibration.resolution_width,
                     calibration.depth_camera_calibration.resolution_height,
                     calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                     &xy_table);

        for (;running;) {
        
            k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS);
            k4a_image_t ir_image = k4a_capture_get_ir_image(capture);
            k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
            k4a_image_t color_image = k4a_capture_get_color_image(capture);
            int height = k4a_image_get_height_pixels(ir_image);
            int width = k4a_image_get_width_pixels(ir_image);
        // int strides = k4a_image_get_stride_bytes(image);
        // printf(" height: %d , %d ", height, width);
            //printf("stride: %d", strides);

            // One way to convert 16bit to 8bit with user specified min/max dynamic range
            uint8_t* imgData = k4a_image_get_buffer(ir_image);
            uint16_t* irImg = reinterpret_cast<uint16_t*>(imgData);
            std::vector<uint8_t> grayScaleImg(width * height);
            int irMinValue = 0;
            int irMaxValue = 1000;
            ConvertToGrayScaleImage(irImg, width * height, irMinValue, irMaxValue, grayScaleImg.data());
            const cv::Mat irImage(cv::Size(width, height), CV_8U, grayScaleImg.data());
        // cv::Mat irImage;
        // createTransformedColorImageToFile(irImage, transformation, depth_image, color_image  );
            cv::imshow("foobar", irImage);
        //  cv::waitKey(0);

            int key = cv::waitKey(10);
            switch(key & 0xFF)
            {
            case 27:
            case 'q':
                running = false;
                break;
            case ' ':
            case 's':
                std::cout<<"Saving IR and depth image "<<serial_number<<std::endl;
                cv::imwrite( std::string(serial_number)+"_ir.png", irImage );

                cv::Mat depthImage(height , width, CV_16UC1, (void *)k4a_image_get_buffer(depth_image));
            //  depthImage.convertTo(depthImage, CV_16UC1); //  CV_32FC1
                cv::imwrite( std::string(serial_number)+"_depth.png", depthImage );
               
                create_xy_table(&calibration, xy_table, serial_number);
                k4a_image_release(xy_table);
                break;

               
 
            }
           

            k4a_image_release(ir_image);
            k4a_image_release(depth_image);
            k4a_capture_release(capture);

        }
    }
    cv::destroyAllWindows();
   // cv::waitKey(100);

    k4a_device_close(device);
    return 0;
}