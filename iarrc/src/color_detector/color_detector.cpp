#include "color_detector.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace ros;
using namespace cv;

namespace iarrc {

    const Scalar blue_low{78, 50, 70};
    const Scalar blue_high{138, 255, 255};
    const Scalar blue_label{255, 0, 0};

    const Scalar white_low{0, 00, 175}; //10->25
    const Scalar white_high{255, 32, 255}; //32->62
    const Scalar white_label{255, 255, 255};

    const Scalar orange_low{0, 80, 40};
    const Scalar orange_high{30, 255, 255};
    const Scalar orange_label{0, 127, 255};

    const Scalar yellow_low{25, 45, 45}; //55,50 -> 65, 23->33
    const Scalar yellow_high{45, 255, 255}; //47 > 57
    const Scalar yellow_label{0, 255, 255};

    void color_detector::ImageCB(const sensor_msgs::ImageConstPtr &msg) {

        cv_bridge::CvImageConstPtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("CV-Bridge error: %s", e.what());
            return;
        }

        const Mat &frameBGR = cv_ptr->image;
        Mat frameHSV = Mat::zeros(frameBGR.rows, frameBGR.cols, CV_8UC3);
        cvtColor(frameBGR, frameHSV, CV_BGR2HSV);
        const Mat frame_masked = frameHSV(mask);

        Mat output_blue = Mat::zeros(mask.height, mask.width, CV_8U);
        Mat output_white = Mat::zeros(mask.height, mask.width, CV_8U);
        Mat output_orange = Mat::zeros(mask.height, mask.width, CV_8U);
        Mat output_yellow = Mat::zeros(mask.height, mask.width, CV_8U);

        inRange(frame_masked, blue_low, blue_high, output_blue);
        inRange(frame_masked, white_low, white_high, output_white);
        inRange(frame_masked, orange_low, orange_high, output_orange);
        inRange(frame_masked, yellow_low, yellow_high, output_yellow);

        erode(output_blue, output_blue, erosion_kernel_blue);
        erode(output_white, output_white, erosion_kernel_white);
        erode(output_orange, output_orange, erosion_kernel_orange);
        erode(output_yellow, output_yellow, erosion_kernel_yellow);

        Mat output = Mat::zeros(frameHSV.rows, frameHSV.cols, CV_8UC3);
        Mat output_masked = output(mask);

        output_masked.setTo(yellow_label, output_yellow);
        //output_masked.setTo(orange_label, output_orange);
        output_masked.setTo(white_label, output_white);
        output_masked.setTo(blue_label, output_blue);

        img_pub.publish(cv_bridge::CvImage{std_msgs::Header(), "bgr8", output}.toImageMsg());
    }

    void color_detector::onInit() {
        NodeHandle nh = getNodeHandle();
        NodeHandle pnh = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);

        int mask_y_coordinate;
        pnh.param("mask_y_coordinate", mask_y_coordinate, 280);

        mask = Rect(0, mask_y_coordinate, 640, 480-mask_y_coordinate); // x, y, w, h

        erosion_kernel_blue = getStructuringElement(MORPH_ELLIPSE, Size(11, 11));
        erosion_kernel_white = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
        erosion_kernel_orange = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
        erosion_kernel_yellow = getStructuringElement(MORPH_ELLIPSE, Size(7, 7)); //9->5

        img_sub = it.subscribe("/camera/image_rect", 1, &color_detector::ImageCB, this);
        img_pub = it.advertise("/colors_img", 1);

        ROS_INFO("Color Detector ready!");
    }

}

PLUGINLIB_EXPORT_CLASS(iarrc::color_detector, nodelet::Nodelet)