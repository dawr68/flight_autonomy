#include <ros/ros.h>
#include "FlightAutonomy/FlightAutonomy.h"

// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>

// cv_bridge::CvImagePtr _camera_view = nullptr;

// std::string OPENCV_WINDOW = "hello";

// class ImageConverter
// {
//   ros::NodeHandle _nh;
//   ros::Subscriber image_sub_;

// public:
//   ImageConverter()
//   {
//     // Subscrive to input video feed and publish output video feed
//     image_sub_ = _nh.subscribe("/iris_race/c920/image_raw", 1,
//       &ImageConverter::imageCb, this);
//     cv::namedWindow(OPENCV_WINDOW);
//   }

//   ~ImageConverter()
//   {
//     cv::destroyWindow(OPENCV_WINDOW);
//   }

//   void imageCb(const sensor_msgs::ImageConstPtr& msg)
//   {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }

//     // Draw an example circle on the video stream
//     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//       cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

//     // Update GUI Window
//     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//     cv::waitKey(3);

//     // Output modified video stream

//   }
// };


// void img_sub_callback(sensor_msgs::ImageConstPtr &img)
// {
//     try
//     {
//         _camera_view = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//         ROS_ERROR("cv_bridge error at img copy: %s", e.what());
//     }
// }


int main(int argc, char **argv)
{

    ros::init(argc, argv, "flight_autonomy");
    ros::NodeHandle nh("~");
    ros::Rate rate(30);

    FlightAutonomy fliAuto(nh);

    ros::spin();
    return 0;
}