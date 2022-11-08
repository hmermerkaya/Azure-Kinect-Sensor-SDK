/*
 * multiKinectv2GrabberROS.cpp
 *
 *  Created on: Nov 28, 2018
 *      Author: hamit
 */






// extra headers for writing out ply file
//#include <pcl/console/print.h>
//#include <pcl/console/parse.h>
//#include <pcl/console/time.h>
//#include <pcl/io/ply_io.h>
#include "k4a/k4a.h"
#include "k4AWithROS2.h"
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/program_options.hpp>

#include <thread>
#include <iostream>

long now_ms(void) {

	struct timespec res;
	clock_gettime(CLOCK_REALTIME, &res);
	return 1000 * res.tv_sec +  res.tv_nsec / 1e6;

}

long getMicrotime(){
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	return currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;
}

std::vector<std::string> getk4ASerials(){
    std::vector<std::string> serials;

    k4a_device_t device = NULL;

    uint32_t device_count = k4a_device_get_installed_count();
    printf("Found %d connected devices:\n", device_count);
    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
        {
            printf("%d: Failed to open device\n", deviceIndex);
        }

        char *serial_number = NULL;
        size_t serial_number_length = 0;

        if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(device, NULL, &serial_number_length))
        {
            printf("%d: Failed to get serial number length\n", deviceIndex);
            k4a_device_close(device);
            device = NULL;
            continue;
        }


        serial_number = (char*)malloc(serial_number_length);
        if (serial_number == NULL)
        {
            printf("%d: Failed to allocate memory for serial number (%zu bytes)\n", deviceIndex, serial_number_length);
            k4a_device_close(device);
            device = NULL;
            continue;
        }

        if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(device, serial_number, &serial_number_length))
        {
            printf("%d: Failed to get serial number\n", deviceIndex);
            free(serial_number);
            serial_number = NULL;
            k4a_device_close(device);
            device = NULL;
            continue;
        }
        serials.push_back(serial_number);
        printf("%d: Device \"%s\"\n", deviceIndex, serial_number);

        free(serial_number);
        serial_number = NULL;
        k4a_device_close(device);
        device = NULL;
    }

    return  serials;

}


int main(int argc, char * argv[])
{

	namespace po = boost::program_options;
	std::vector<std::string> serials=getk4ASerials();
	

	long postime;
	bool fullhd;
	try {
		po::options_description desc("");
		desc.add_options()
	//	 ("serials,s", po::value<std::vector<std::string> >()->required()->multitoken(), "kinectv2 serials")
		 ("fullhd", po::value<bool>()->default_value(true), "full hd")
		 ("postime", po::value<long>()->required(), "the time to start grabber");
		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);



	//	serials = vm["serials"].as<std::vector<std::string> >();
		if (vm.count("postime")) postime=vm["postime"].as<long>();
		 else {
		    std::cout << "Post time  was not set. \n";
		}
		fullhd=vm["fullhd"].as<bool>();

	}
	catch (const po::error &ex)
	  {
	    std::cerr << ex.what() << '\n';
	   // if (serials.size()==0) PCL_ERROR ("Any kinectv2 serial is not provided! Please enter serial to continue. Syntax is %s --serials serial1 serial2 ...\n", argv[0]);
	    exit(1);

	 }

	//std::cout<<"fullhd "<<fullhd<<std::endl;
	bool mirroring = true;
    const int kinect_count=serials.size();
	std::vector<k4AWithROS* > kinects;
	ros::init(argc, argv, kinect_count==2?"k4A_"+serials[0]+"_"+serials[1]:"k4A_"+serials[0]);

//    for (auto &k:kinects) {
//
//    	 k= new K2GRos(freenectprocessor, mirroring, serials[j]);
//         std::cout<<serials[j]<<" device started"<<std::endl;
//         ++j;
//     }



    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    k4a_device_t devices[kinect_count];
   
    for (int i=0;i<kinect_count;i++)  kinects.push_back(new k4AWithROS(i, serials[i], config, devices[i]));

    


 //   boost::posix_time::ptime posixNexTime = boost::asio::time_traits<boost::posix_time::ptime>::now();

   boost::posix_time::ptime time_epoch(boost::gregorian::date(1970, 1, 1));
   boost::posix_time::ptime posixNexTime = time_epoch + boost::posix_time::milliseconds(postime+3000);

//    cout<<"posixNexTime "<<posixNexTime<< " "<<t<<std::endl;

    long startTime[3]={0}, endTime[3]={0};
    int sumendTimeDeltas = 0,sumstartTimeDeltas = 0, sumstartendTimeDeltas=0;
    int cnt=0, realcnt=0;

    bool flag_exit=false;

	while((ros::ok()))
	{
		int i=0;
		boost::asio::io_service io;
		std::vector< boost::asio::deadline_timer*> timers;
		posixNexTime += boost::posix_time::time_duration(boost::posix_time::milliseconds(100));
       		ros::Time tm = ros::Time::now();

		for (const auto &k: kinects) {

			if (0)
					{
						flag_exit=true;

						break;
					}





			 boost::asio::deadline_timer *timer  = new boost::asio::deadline_timer(io);
			 timer->expires_at(posixNexTime);
			 timer->async_wait([=, &startTime, &endTime, &tm] (const boost::system::error_code&){


				   // startTime[i] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
				    startTime[i] = getMicrotime();//now_ms();
				//    std::cout<<"startTime[i]: "<<i<<" "<<now_ms()<<std::endl;
				  //  ros::Duration dura( 0.01*((float)rand())/RAND_MAX);
				//    tm+=dura;
				    k->publishDepthColorCompressedAndCameraInfos(ros::Time::now(),fullhd);
//					k->publishCameraInfoDepth();
//
				    endTime[i] = getMicrotime();//now_ms();
				//	endTime[i] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
//					std::cout<<"Delta start end : "<<i<<" "<<endTime[i]-startTime[i]<<std::endl;



			   });



			  timers.push_back(timer);
			  ++i;

		}
		if (flag_exit) break;

		std::vector<std::thread > ths;

		for (int l=0;l<kinect_count; ++l) ths.push_back (std::thread([&](){io.run();}));

		for (auto &t: ths) t.join();
//		sumstartTimeDeltas=sumstartTimeDeltas + (1.f/1)*sqrt((startTime[0]- startTime[1])*(startTime[0]- startTime[1])+ (startTime[0]- startTime[2])*(startTime[0]- startTime[2]) + (startTime[1]- startTime[2])*(startTime[1]- startTime[2])) ;
//		sumendTimeDeltas=sumendTimeDeltas + (1.f/1)*sqrt((endTime[0]- endTime[1])*(endTime[0]- endTime[1])+ (endTime[0]-endTime[2])*(endTime[0]- endTime[2]) + (endTime[1]- endTime[2])*(endTime[1]- endTime[2])) ;
         if (cnt>30) {
			sumstartTimeDeltas=sumstartTimeDeltas + (1.f/1)*sqrt((startTime[0]- startTime[1])*(startTime[0]- startTime[1])) ;
			sumendTimeDeltas=sumendTimeDeltas + (1.f/1)*sqrt((endTime[0]- endTime[1])*(endTime[0]- endTime[1])) ;
		//	sumstartendTimeDeltas=sumstartendTimeDeltas + (1.f/1)*sqrt((endTime[0]- startTime[0])*(endTime[0]- startTime[0])+ (endTime[1]- startTime[1])*(endTime[1]- startTime[1])) ;
			sumstartendTimeDeltas=sumstartendTimeDeltas + (abs(endTime[0]- startTime[0]) + abs(endTime[1]- startTime[1]))/kinect_count ;

			realcnt++;
         }
		cnt++;
	}
		 std::cout<<"Average start Delta: "<<float(sumstartTimeDeltas)/realcnt/1000.0<<std::endl;
		 std::cout<<"Average end Delta: "<<float(sumendTimeDeltas)/realcnt/1000.0<<std::endl;
	     std::cout<<"Average Delta start end : "<<float(sumstartendTimeDeltas)/realcnt/1000.0<<std::endl;




	for (auto &k:kinects) k->shutdown();
	return 0;
}


