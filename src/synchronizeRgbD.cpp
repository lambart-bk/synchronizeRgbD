#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>    

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui.hpp>

#include"zlib.h"
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread/condition_variable.hpp>

#include"iostream"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace cv;
using namespace std;


FILE *logfile=NULL;
int32_t numFrames=0;

int height=480;
int width=640;
CvMat * encodedImage=NULL;
int   depth_compress_buf_size = width * height * sizeof(int16_t) * 4;
uint8_t *   depth_compress_buf = (uint8_t*)malloc(depth_compress_buf_size);

void encodeJpeg(cv::Vec<unsigned char, 3> * rgb_data)
{
    cv::Mat3b rgb(height, width, rgb_data, width * 3);

    IplImage * img = new IplImage(rgb);

    int jpeg_params[] = {CV_IMWRITE_JPEG_QUALITY, 90, 0};

    if(encodedImage != 0)
    {
        cvReleaseMat(&encodedImage);
    }

    encodedImage = cvEncodeImage(".jpg", img, jpeg_params);

    delete img;
}


void callback_rgbd(   const sensor_msgs::ImageConstPtr &img_rgb ,
                      const sensor_msgs::ImageConstPtr &img_dpt  )
{
    //ROS_INFO("rgb stamp: %f", img_rgb->header.stamp.toSec());
    //ROS_INFO(" d  stamp: %f\n", img_dpt->header.stamp.toSec());	
    
    try
    {
      //std::cout<<"rgb encoding:\t"<<img_rgb->encoding<<std::endl;
      //std::cout<<"  d encoding:\t"<<img_dpt->encoding<<std::endl;
      cv_bridge::CvImagePtr cv_img_rgb = cv_bridge::toCvCopy(img_rgb, sensor_msgs::image_encodings::BGR8);
      cv_bridge::CvImagePtr cv_img_d   = cv_bridge::toCvCopy(img_dpt, sensor_msgs::image_encodings::TYPE_32FC1);
      
      cv::Mat mat_rgb   = cv_img_rgb->image.clone();
      cv::Mat mat_d_raw =   cv_img_d->image.clone();
      
      cv::Mat mat_dpt(mat_d_raw.rows,mat_d_raw.cols,CV_16UC1);
      double scale=1000.0;
      for(int i=0;i<mat_d_raw.rows;i++)
	for(int j=0;j<mat_d_raw.cols;j++)
	{
	  float ov=mat_d_raw.at<float>(i,j);
	  unsigned short nv=ov*scale;
  	  //std::cout<<"\t"<<nv<<" ";//std::endl;
	  mat_dpt.at<unsigned short>(i,j)=nv;
	} 
	
	
      int64_t timestamp=img_rgb->header.stamp.toNSec();  
//       std::cout<<"timestamp:\t"<<timestamp<<std::endl;
      unsigned char * rgbData = 0;
      unsigned char * depthData = 0;
      unsigned long depthSize = depth_compress_buf_size;
      int32_t rgbSize = 0;
	
      boost::thread_group threads;
      threads.add_thread(new boost::thread(compress2,
					    depth_compress_buf,
					    &depthSize,
					    (const Bytef*)mat_dpt.data,
					    width * height * sizeof(short),
					    Z_BEST_SPEED));
      threads.add_thread(new boost::thread(boost::bind(&encodeJpeg,
							(cv::Vec<unsigned char, 3> *)mat_rgb.data)));
      threads.join_all();

      rgbSize = encodedImage->width;
      depthData = (unsigned char *)depth_compress_buf;
      rgbData = (unsigned char *)encodedImage->data.ptr;
      
      fwrite(&timestamp,sizeof(int64_t),1,logfile);
      fwrite((int32_t *)&depthSize,sizeof(int32_t),1,logfile);
      fwrite(&rgbSize,sizeof(int32_t),1,logfile);
      fwrite(depthData,depthSize,1,logfile);
      fwrite(rgbData,rgbSize,1,logfile);
      numFrames++;
         
    }
    catch(cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge  Exception %s",e.what());
    }
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "synchronizeRgbD_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ROS_INFO("synchronizeRgbD_node running ...");
    
    std::string filename="/home/yushan/Desktop/datasets/rgbd_dataset_freiburg3_long_office_household.klg";
//     std::string filename="/home/yushan/Desktop/datasets/rgbd_dataset_freiburg1_desk.klg";
    std::cout<<"log in "<<filename<<std::endl;
    logfile=fopen(filename.c_str(),"wb+");
    numFrames=0;
    fwrite(&numFrames,sizeof(int32_t),1,logfile);
    
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb(nh, "/camera/rgb/image_color", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_dpt(nh, "/camera/depth/image", 100);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy_img;
    message_filters::Synchronizer<MySyncPolicy_img> sync_img(MySyncPolicy_img(100), sub_rgb, sub_dpt);
    sync_img.registerCallback(boost::bind(&callback_rgbd, _1, _2));

    ros::Rate loop_rate(30);
    while (nh.ok()) {
        //pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    std::cout<<"numFrames:\t"<<numFrames<<std::endl;
    
    fseek(logfile,0,SEEK_SET);
    fwrite(&numFrames,sizeof(int64_t),1,logfile);
    fflush(logfile);
    fclose(logfile);   
    std::cout<<"fwrite finished !"<<std::endl;
    
    ros::shutdown();
    
    return 0;

}

