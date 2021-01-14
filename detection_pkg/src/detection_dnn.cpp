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
    //image_transport::Subscriber subdepth;

  ImageConverter()
    : it(nh)
  {
   sub  = it.subscribe("/D435I/color/image_raw", 1, &ImageConverter::imageCallback,this);
   //subdepth = it.subscribe("/D435I/depth/image_rect_raw", 1, &ImageConverter::imageCallback,this,image_transport::TransportHints("compressedDepth"));
   pub = it.advertise("/image_converter/output_video", 1);


  }

    ~ImageConverter()
    {
      cv::destroyWindow("detection_window");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {cv_bridge::CvImagePtr img_ptr;//,img_ptr_d;


      try
      {
        
            img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        

      }
      catch (cv_bridge::Exception& e)
      {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
      }
     
     cv::dnn::Net dnnnet = dnn::readNetFromCaffe("/home/abhishek/catkin_ws/src/detection_pkg/include/MobileNetSSD_deploy.prototxt.txt","/home/abhishek/catkin_ws/src/detection_pkg/include/MobileNetSSD_deploy.caffemodel");
     
     cv::Mat blob_input_data = cv::dnn::blobFromImage(img_ptr->image, Scaling_factor,cv::Size(Width, Height), Mean_Value, false);
     
     dnnnet.setInput(blob_input_data,"data");//network input
     Mat output_detection_ = dnnnet.forward("detection_out");//output

     Mat output_detection_mat(output_detection_.size[2], output_detection_.size[3], CV_32F, output_detection_.ptr<float>());

     float confidence_threshold = 0.90f;
             for(int i = 0; i < output_detection_mat.rows; i++)
             {
                 float confidence_output = output_detection_mat.at<float>(i, 2);

                 if(confidence_output > confidence_threshold)
                 {
                     

                     int Left_bottom_xval = static_cast<int>(output_detection_mat.at<float>(i, 3) * img_ptr->image.cols);
                     int Left_bottom_yval = static_cast<int>(output_detection_mat.at<float>(i, 4) * img_ptr->image.rows);
                     int Right_top_xval = static_cast<int>(output_detection_mat.at<float>(i, 5) * img_ptr->image.cols);
                     int Right_top_yval = static_cast<int>(output_detection_mat.at<float>(i, 6) * img_ptr->image.rows);

                     Rect rect_box((int)Left_bottom_xval, (int)Left_bottom_yval,
                                 (int)(Right_top_xval - Left_bottom_xval),
                                 (int)(Right_top_yval - Left_bottom_yval));

                     rect_box = rect_box  & Rect(0, 0, img_ptr->image.cols, img_ptr->image.rows);
                     Scalar m = mean(img_ptr->image(rect_box));

                                     

                                     rectangle(img_ptr->image, rect_box, Scalar(0, 255, 0));
                                     
                    }
                }

             cv::imshow("detection_window", img_ptr->image);
             cv::waitKey(1);
             pub.publish(img_ptr->toImageMsg());

    }




};
int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();

  return 0;

}


