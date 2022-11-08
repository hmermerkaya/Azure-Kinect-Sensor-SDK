find_package(OpenCV     REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )
#set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-comment -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function -Wno-unused-parameter -Wno-maybe-uninitialized")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-sign-compare -Wno-reorder -Wno-deprecated-declarations -Wno-unknown-pragmas -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function -Wno-unused-parameter -Wno-maybe-uninitialized -Wno-unused-result" )

option(WITH_ROS "adds support for ROS" ON)
if(${WITH_ROS})
	find_package(catkin REQUIRED COMPONENTS   roscpp std_msgs pcl_conversions  pcl_ros
						sensor_msgs cv_bridge image_transport tf
						message_generation )
	catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions  pcl_ros
						sensor_msgs cv_bridge image_transport tf
						cmake_modules
	)
	include_directories(${catkin_INCLUDE_DIRS} )
endif()

option(WITH_PCL "adds pcl cloud support" ON)
if(${WITH_PCL})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_PCL")
	find_package(PCL  REQUIRED)
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	include_directories(${PCL_INCLUDE_DIRS})
endif()

#message("catkin include ${catkin_INCLUDE_DIRS}")

add_executable( k4AROSPublisher
    k4AROSPublisher.cpp
    )

target_link_libraries( k4AROSPublisher PRIVATE 
${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}  
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo
    
    )

add_executable( k4AROSViewer
    k4AROSViewer.cpp
    )

target_link_libraries( k4AROSViewer PRIVATE 
${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}  
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo
    
    )

target_compile_features(k4AROSViewer PUBLIC cxx_std_14)




add_executable( k4AROSViewer1
    k4AROSViewer1.cpp
    )

target_link_libraries( k4AROSViewer1 PRIVATE 
${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo

    )

target_compile_features(k4AROSViewer1 PUBLIC cxx_std_14)





add_executable( k4AROSViewer2cams
    k4AROSViewer2cams.cpp
    )

target_link_libraries( k4AROSViewer2cams PRIVATE
${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo
    )

target_compile_features(k4AROSViewer2cams PUBLIC cxx_std_14)


add_executable( k4AROSViewer2
    k4AROSViewer2.cpp
    )

target_link_libraries( k4AROSViewer2 PRIVATE
${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo

    )

target_compile_features(k4AROSViewer2 PUBLIC cxx_std_14)


add_executable( k4AROSViewer3
    k4AROSViewer3.cpp
    )

target_link_libraries( k4AROSViewer3 PRIVATE
${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo

    )

target_compile_features(k4AROSViewer3 PUBLIC cxx_std_14)


add_executable( k4AROSViewer4
    k4AROSViewer4.cpp
    )

target_link_libraries( k4AROSViewer4 PRIVATE
${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo

    )

target_compile_features(k4AROSViewer4 PUBLIC cxx_std_14)

add_executable( k4AROSViewer6
    k4AROSViewer6.cpp
    )

target_link_libraries( k4AROSViewer6 PRIVATE
${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo

    )

target_compile_features(k4AROSViewer6 PUBLIC cxx_std_14)





    add_executable( k4ACapture
    k4ACapture.cpp
    )

target_link_libraries( k4ACapture PRIVATE 
    k4a::k4a  ${OpenCV_LIBS}  
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo
    
    )

target_compile_features(k4ACapture PUBLIC cxx_std_14)



    add_executable(k4ACapture2pcd 
    k4ACapture2pcd.cpp  transformation_helpers.cpp)
    target_link_libraries(k4ACapture2pcd
    k4a::k4a  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${BOOST_LIBS}
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo

    )
#    add_executable(meshlabCams meshlabCams.cpp)
#    target_link_libraries(meshlabCams ${OpenCV_LIBS} ${PCL_LIBRARIES} ${BOOST_LIBS})
#    add_executable(registerPairCams 
#    registerPairCams.cpp ) 
#    target_link_libraries(registerPairCams
#    ${catkin_LIBRARIES} k4a::k4a  ${PCL_LIBRARIES} ${OpenCV_LIBS}  ${BOOST_LIBS} 
#    k4a::k4arecord
#    libjpeg-turbo::libjpeg-turbo

#    )

#    add_executable(transforms   transforms.cpp)
#    target_link_libraries(transforms  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
    add_executable(kinfuCams   kinfuCams.cpp)
    target_link_libraries(kinfuCams  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
    target_compile_features(kinfuCams PUBLIC cxx_std_14)

    #  add_executable( k4AROSSubscriber
    # k4AROSSubscriber.cpp
    #)
    #target_link_libraries( k4AROSSubscriber PRIVATE 
    # ${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}  
    #k4a::k4arecord
    #libjpeg-turbo::libjpeg-turbo
    
    # )

    #target_compile_features(k4AROSSubscriber PUBLIC cxx_std_14)


#    add_executable( k4AROSSubscriber2
#    k4AROSSubscriber2.cpp
#    )
#    target_link_libraries( k4AROSSubscriber2 PRIVATE 
#     ${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS} ${Boost_LIBRARIES}  draco avformat avcodec swscale avutil avfilter swresample avdevice postproc pthread fdk-aac mp3lame  
#     ass theora ogg  x264 x265 vpx vorbis vorbisfile vorbisenc theoraenc theoradec opus  z lzma bz2 rt
#    k4a::k4arecord
#    libjpeg-turbo::libjpeg-turbo

#    )

#    target_compile_features(k4AROSSubscriber2 PUBLIC cxx_std_14)

    #add_executable( k4AROSSubscriber2_6cams
    #k4AROSSubscriber2_6cams.cpp
    #)
    #target_link_libraries( k4AROSSubscriber2_6cams PRIVATE 
    # ${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}  draco avformat avcodec swscale avutil avfilter swresample avdevice postproc pthread fdk-aac mp3lame  
    # ass theora ogg  x264 x265 vpx vorbis vorbisfile vorbisenc theoraenc theoradec opus  z lzma bz2 rt
    #k4a::k4arecord
    #libjpeg-turbo::libjpeg-turbo

    #)

    #add_executable( k4AROSSubscriber2_pointcloud_6cams
    #k4AROSSubscriber2_pointcloud_6cams.cpp
    #)
    #target_link_libraries( k4AROSSubscriber2_pointcloud_6cams PRIVATE 
    # ${catkin_LIBRARIES} k4a::k4a  ${OpenCV_LIBS}  draco avformat avcodec swscale avutil avfilter swresample avdevice postproc pthread fdk-aac mp3lame  
    # ass theora ogg  x264 x265 vpx vorbis vorbisfile vorbisenc theoraenc theoradec opus  z lzma bz2 rt
    #k4a::k4arecord
    #libjpeg-turbo::libjpeg-turbo

    #)
