#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Detection Window";
//#include <string>
//#include <sstream>

//namespace patch
//{
//    template < typename T > std::string to_string( const T& n )
//    {
//        std::ostringstream stm ;
//        stm << n ;
//        return stm.str() ;
//    }
//}
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Subscriber image_sub_depth;
  image_transport::Publisher image_pub_;
  

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/D435I/color/image_raw", 1,
      &ImageConverter::imageCb, this);
    //image_sub_depth = it_.subscribe("/D435I/depth/image_rect_raw", 1,
     // &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){

        HOGDescriptor hog_desc;
        hog_desc.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	
            vector<Rect> identify, identified_filtered;
            hog_desc.detectMultiScale(cv_ptr->image, identify, 0, Size(8,8), Size(32,32), 1.05, 2);

            size_t a, b;
            for (a=0; a<identify.size(); a++)
            {
                Rect rect = identify[a];
                for (b=0; b<identify.size(); b++)
                    if (b!=a && (rect & identify[b])==rect)
                        break;
                if (b==identify.size())
                    identified_filtered.push_back(rect);
            }
            for (a=0; a<identified_filtered.size(); a++)
            {
                Rect rect = identified_filtered[a];
                rect.x += cvRound(rect.width*0.1);
                rect.width = cvRound(rect.width*0.8);
                rect.y += cvRound(rect.height*0.06);
                rect.height = cvRound(rect.height*0.9);
                rectangle(cv_ptr->image, rect.tl(), rect.br(), cv::Scalar(0,255,0), 2);
//		float pixel_distance;
//		// TODO:  calculate the median of all the values inside the box region
//                pixel_distance = 0.001*(cv_ptr->image.at<u_int16_t>(rect.tl(),rect.br()));
//                cv::putText(cv_ptr->image,patch::to_string(pixel_distance),cv::Point2f(rect.tl(),rect.br()),cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255,0,0), 2);
//                cv::circle(cv_ptr->image,cv::Point2f(rect.br()+10,rect.tl()),10,cv::Scalar(0),2);
            }

	
	


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
