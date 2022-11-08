#include <iostream>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <string>
#include "turbojpeg.h"
#include <fstream>
#include <assert.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv.h>
#include <highgui.h>
using namespace cv;

static long WriteToFile(const char *fileName, void *buffer, size_t bufferSize)
{
    std::cout << bufferSize << std::endl;
    assert(buffer != NULL);

    std::ofstream hFile;
    hFile.open(fileName, std::ios::out | std::ios::trunc | std::ios::binary);
    if (hFile.is_open())
    {
        hFile.write((char *)buffer, static_cast<std::streamsize>(bufferSize));
        hFile.close();
    }
    std::cout << "[Streaming Service] Color frame is stored in " << fileName << std::endl;

    return 0;
}


static bool WriteTransfomedColorImageToFile(std::string file_name, k4a_transformation_t transformation_handle,
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
        return false;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                                depth_image,
                                                                                color_image,
                                                                                transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }

    
    // int color_width, color_height;
    // color_width = k4a_image_get_width_pixels(transformed_color_image);
    // color_height = k4a_image_get_height_pixels(transformed_color_image);

    // if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
    //                                              color_width,
    //                                              color_height,
    //                                              color_width * 4 * (int)sizeof(uint8_t),
    //                                              &uncompressed_transformed_color_image))
    // {
    //     printf("Failed to create image buffer\n");
        
    // }

    // tjhandle tjHandle;
    // tjHandle = tjInitDecompress();
    // if (tjDecompress2(tjHandle,
    //                   k4a_image_get_buffer(transformed_color_image),
    //                   static_cast<unsigned long>(k4a_image_get_size(transformed_color_image)),
    //                   k4a_image_get_buffer(uncompressed_transformed_color_image),
    //                   color_width,
    //                   0, // pitch
    //                   color_height,
    //                   TJPF_BGRA,
    //                   TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
    // {
    //     printf("Failed to decompress color frame\n");
    //     if (tjDestroy(tjHandle))
    //     {
    //         printf("Failed to destroy turboJPEG handle\n");
    //     }
       
    // }
    // if (tjDestroy(tjHandle))
    // {
    //     printf("Failed to destroy turboJPEG handle\n");
    // }



    // std::ofstream file;
    // file.open(file_name, std::ios::out | std::ios::trunc | std::ios::binary);
    // if (file.is_open())
    // {
    //     file.write((char *)(void *)k4a_image_get_buffer(transformed_color_image), 
    //     static_cast<std::streamsize>(k4a_image_get_size(transformed_color_image)));
    //     file.close();
    // } else return false;


    int rows = k4a_image_get_height_pixels(transformed_color_image);
    int cols = k4a_image_get_width_pixels(transformed_color_image);
    cv::Mat colorMat(rows , cols, CV_8UC4, (void *)k4a_image_get_buffer(transformed_color_image), cv::Mat::AUTO_STEP);
    cv::Mat gray_image;
    cvtColor( colorMat, gray_image, CV_BGRA2GRAY );
    cv::imwrite( "./transformed.png", colorMat );
    std::cout << "[Streaming Service] Transformed Color frame is stored in " << file_name << std::endl;
    return true;
}
#define CloseAndReturn() \
    if (depth_image != NULL) \
    {\
        k4a_image_release(depth_image);\
    }\
    if (color_image != NULL)\
    {\
        k4a_image_release(color_image);\
    }\
    if (capture != NULL)\
    {\
        k4a_capture_release(capture);\
    }\
    if (transformation != NULL)\
    {\
        k4a_transformation_destroy(transformation);\
    }\
    if (device != NULL)\
    {\
        k4a_device_close(device);\
    } \
    return returnCode;


int main(void) {

    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    k4a_transformation_t transformation = NULL;
    k4a_capture_t capture = NULL;
    std::string file_name = "";
    uint32_t device_count = 0;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t ir_image = NULL;
    device_count = k4a_device_get_installed_count();



    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }
    uint8_t deviceId = K4A_DEVICE_DEFAULT;
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceId, &device))
    {
        printf("Failed to open device\n");
        CloseAndReturn();
    }


    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;//K4A_IMAGE_FORMAT_COLOR_MJPG;//
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;//K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.synchronized_images_only = true; 



    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        CloseAndReturn();
    }

    transformation = k4a_transformation_create(&calibration);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras\n");
        CloseAndReturn();
    }
    // Get a capture
    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        CloseAndReturn();
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        CloseAndReturn();
    }
    // Get a depth image
    depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0)
    {
        printf("Failed to get depth image from capture\n");
        CloseAndReturn();
    }

    // Get a color image
    color_image = k4a_capture_get_color_image(capture);
    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
        CloseAndReturn();
    }
    ir_image = k4a_capture_get_ir_image(capture);

    int rows = k4a_image_get_height_pixels(ir_image);
    int cols = k4a_image_get_width_pixels(ir_image);
    cv::Mat colorMat(rows , cols, CV_8UC1, (void *)k4a_image_get_buffer(ir_image), cv::Mat::AUTO_STEP);
    // cv::Mat gray_image;
    // cvtColor( colorMat, gray_image, CV_BGRA2GRAY );
    cv::imwrite( "./ir.png", colorMat );

    WriteToFile("test_color.jpg", k4a_image_get_buffer(color_image), k4a_image_get_size(color_image));
    WriteToFile("test_depth.bin", k4a_image_get_buffer(depth_image), k4a_image_get_size(depth_image));
    WriteTransfomedColorImageToFile("testTransformed_color.jpg", transformation, depth_image, color_image  );
    
    returnCode=0;

return returnCode;    
}