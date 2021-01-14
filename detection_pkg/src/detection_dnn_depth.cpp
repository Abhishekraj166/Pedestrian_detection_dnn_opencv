#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;
using namespace cv::dnn;


using namespace message_filters;
const size_t Width = 320;
const size_t Height = 320;
const float window_ratio       = Width / (float)Height;
const float Scaling_factor = 0.007843f;
const float Mean_Value       = 127.5;
const char* classNames[] = {"person"};




class ImageConverter
{public:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher pub;
    image_transport::Subscriber sub;
    image_transport::Subscriber subdepth;

  ImageConverter()
    : it(nh)
  {
      sub= it.subscribe("/D435I/color/image_raw", 1,&ImageConverter::imageCallback, this);
      subdepth= it.subscribe("/D435I/depth/image_rect_raw", 1,&ImageConverter::imageCallback, this,image_transport::TransportHints("compressedDepth"));
      pub = it.advertise("/image_converter/output_video", 1);


  }

    ~ImageConverter()
    {
      cv::destroyWindow("detection_window");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg_rgb,const sensor_msgs::ImageConstPtr& msg_depth)
    {cv_bridge::CvImagePtr img_ptr_rgb;
     cv_bridge::CvImagePtr img_ptr_depth;


      try
      {
        
            img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::BGR8);
        

      }
      catch (cv_bridge::Exception& e)
      {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
      }
     try
     {

           img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::MONO8);


     }
     catch (cv_bridge::Exception& e)
     {
           ROS_ERROR("cv_bridge exception: %s", e.what());
           return;
     }
     cv::dnn::Net dnnnet = dnn::readNetFromCaffe("/home/abhishek/catkin_ws/src/detection_pkg/include/MobileNetSSD_deploy.prototxt.txt","/home/abhishek/catkin_ws/src/detection_pkg/include/MobileNetSSD_deploy.caffemodel");

     cv::Mat& depth_mat = img_ptr_depth->image;
     depth_mat.convertTo(depth_mat,CV_64F);


     cv::Mat& rgb_mat = img_ptr_rgb->image;

     cv::Mat blob_input_data = cv::dnn::blobFromImage(rgb_mat, Scaling_factor,cv::Size(Width, Height), Mean_Value, false);

     dnnnet.setInput(blob_input_data,"data");//network input
     Mat output_detection_ = dnnnet.forward("detection_out");//output

     Mat output_detection_mat(output_detection_.size[2], output_detection_.size[3], CV_32F, output_detection_.ptr<float>());


     float confidence_threshold = 0.90f;
             for(int i = 0; i < output_detection_mat.rows; i++)
             {
                 float confidence_output = output_detection_mat.at<float>(i, 2);

                 if(confidence_output > confidence_threshold)
                 {
                     
                     size_t object_Class = (size_t)( output_detection_mat.at<float>(i,1));
                     int Left_bottom_xval = static_cast<int>(output_detection_mat.at<float>(i, 3) * rgb_mat.cols);
                     int Left_bottom_yval = static_cast<int>(output_detection_mat.at<float>(i, 4) * rgb_mat.rows);
                     int Right_top_xval = static_cast<int>(output_detection_mat.at<float>(i, 5) * rgb_mat.cols);
                     int Right_top_yval = static_cast<int>(output_detection_mat.at<float>(i, 6) * rgb_mat.rows);

                     Rect rect_box((int)Left_bottom_xval, (int)Left_bottom_yval,
                                 (int)(Right_top_xval - Left_bottom_xval),
                                 (int)(Right_top_yval - Left_bottom_yval));


                     rect_box = rect_box  & Rect(0, 0, depth_mat.cols,depth_mat.rows);

                     Scalar m = mean(depth_mat(rect_box));
                     std::ostringstream ss;
                     ss << classNames[object_Class]<<" ";
                     ss << std::setprecision(2) << m[0] << " meters away";
                     String conf(ss.str());


                     rectangle(rgb_mat, rect_box, Scalar(0, 255, 0));
                     int baseLine = 0;
                     Size labelSize = getTextSize(ss.str(), FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

                     cv::Point_<int> center = (rect_box.br() + rect_box.tl())*0.5;
                     center.x = center.x - labelSize.width / 2;

                     rectangle(rgb_mat, Rect(Point(center.x, center.y - labelSize.height),
                     Size(labelSize.width, labelSize.height + baseLine)),
                     Scalar(255, 255, 255), FILLED);
                     putText(rgb_mat, ss.str(), center,
                     FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
                                     
                    }
                }

             cv::imshow("detection_window", rgb_mat);
             cv::waitKey(1);
             pub.publish(img_ptr_rgb->toImageMsg());

    }




};
int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_converter");

  ImageConverter ic;

  ros::spin();

  return 0;

}


