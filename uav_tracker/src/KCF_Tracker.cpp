#include "KCF_Tracker.h"

Rect selectRect;
Point origin;
Rect result;
bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
Mat rgbimage;
Mat depthimage;
float minDist = 0;
const int &ACTION_ESC = 27;

void onMouse(int event, int x, int y, int, void *)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);
        selectRect.height = abs(y - origin.y);
        selectRect &= Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }

    if (event == EVENT_LBUTTONDOWN)
    {
        bBeginKCF = false;
        select_flag = true;
        minDist = 0;
        origin = Point(x, y);
        selectRect = Rect(x, y, 0, 0);
    }
    else if (event == EVENT_LBUTTONUP)
    {
        select_flag = false;
        bRenewROI = true;
    }
}

ImageConverter::ImageConverter(ros::NodeHandle &n)
{
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
    // Subscrive to input video feed and publish output video feed
    image_sub_ = n.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
    depth_sub_ = n.subscribe("/camera/depth/image_raw", 1, &ImageConverter::depthCb, this);

    target_pub_ = n.advertise<geometry_msgs::Point>("/tracker/target_position", 1);
    status_pub_ = n.advertise<std_msgs::Bool>("/tracker/is_tracking", 1);
    
    namedWindow(RGB_WINDOW);
    namedWindow(DEPTH_WINDOW);
}

ImageConverter::~ImageConverter()
{
    n.shutdown();
    image_sub_.shutdown();
    depth_sub_.shutdown();
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;
    destroyWindow(RGB_WINDOW);
    destroyWindow(DEPTH_WINDOW);
}

void ImageConverter::Reset() {
    bRenewROI = false;
    bBeginKCF = false;
    selectRect.x = 0;
    selectRect.y = 0;
    selectRect.width = 0;
    selectRect.height = 0;
    enable_get_depth = false;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    int center_x = result.x + result.width / 2;
    int center_y = result.y + result.height / 2;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(rgbimage);
    setMouseCallback(RGB_WINDOW, onMouse, 0);

    if (bRenewROI)
    {
        if (selectRect.width <= 0 || selectRect.height <= 0)
        {
            bRenewROI = false;
            return;
        }
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }

    std_msgs::Bool is_tracking_msg;
    geometry_msgs::Point target_pos_msg;

    if (bBeginKCF)
    {
        enable_get_depth = true;
        result = tracker.update(rgbimage);
        rectangle(rgbimage, result, Scalar(0, 255, 255), 1, 8);
        circle(rgbimage, Point(center_x, center_y), 3, Scalar(0, 0, 255), -1);
        string text_x;
        string text_y;
        string text_z;
        text_x.append("X: ");
        text_x.append(to_string(center_x));
        text_y.append("Y: ");
        text_y.append(to_string(center_y));
        text_z.append("Z: ");

        if ((double)minDist < 0.0)
        {
            text_z.append("inf");
        }
        else 
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << (double)(minDist);
            text_z.append(oss.str());
        }
        text_z.append("m");
        putText(rgbimage, text_x.c_str(), Point(center_x + 10, center_y - 13), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(0, 0, 255), 1, 8);
        putText(rgbimage, text_y.c_str(), Point(center_x + 10, center_y + 5), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(0, 255, 0), 1, 8);
        putText(rgbimage, text_z.c_str(), Point(center_x + 10, center_y + 23), FONT_HERSHEY_SIMPLEX, 0.5,
                Scalar(255, 0, 0), 1, 8);
        
        // 计算相对于图像中心的偏差
        float error_x = (float)center_x - (rgbimage.cols / 2.0);
        float error_y = (float)center_y - (rgbimage.rows / 2.0);
        
        target_pos_msg.x = error_x;   // 像素误差
        target_pos_msg.y = error_y;   // 像素误差
        target_pos_msg.z = minDist;   // 深度距离 (m)

        // 简单的可信度检查：如果深度无限远或为0，可能丢失
        if(minDist > 0.1 && minDist < 10.0) {
            is_tracking_msg.data = true;
        } else {
            is_tracking_msg.data = false; // 深度异常视为丢失
        }
    }
    else
    {
        rectangle(rgbimage, selectRect, Scalar(0, 0, 255), 2, 8, 0);

        is_tracking_msg.data = false;
        target_pos_msg.x = 0;
        target_pos_msg.y = 0;
        target_pos_msg.z = 0;
    }

    status_pub_.publish(is_tracking_msg);
    if(is_tracking_msg.data) {
        target_pub_.publish(target_pos_msg);
    }

    imshow(RGB_WINDOW, rgbimage);
    int action = waitKey(1) & 0xFF;

    if (action == 'q' || action == ACTION_ESC) this->Cancel();
    else if (action == 'r') this->Reset();
}

void ImageConverter::depthCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.copyTo(depthimage);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
    }

    if (enable_get_depth)
    {
        minDist = 0;
        int center_x = (int) (result.x + result.width / 2);
        int center_y = (int) (result.y + result.height / 2);
        dist_val[0] = depthimage.at<float>(center_y - 5, center_x - 5)/1000;
        dist_val[1] = depthimage.at<float>(center_y - 5, center_x + 5)/1000;
        dist_val[2] = depthimage.at<float>(center_y + 5, center_x + 5)/1000;
        dist_val[3] = depthimage.at<float>(center_y + 5, center_x - 5)/1000;
        dist_val[4] = depthimage.at<float>(center_y, center_x)/1000;
        int num_depth_points = 5;
        for (int i = 0; i < 5; i++)
        {
            if (dist_val[i] > 0.3 && dist_val[i] < 10.0) minDist += dist_val[i];
            else num_depth_points--;
        }
        minDist /= num_depth_points;
    }
    Mat depthimage_uint8;
    normalize(depthimage, depthimage_uint8, 0, 255, NORM_MINMAX);
    convertScaleAbs(depthimage_uint8, depthimage_uint8);
    imshow(DEPTH_WINDOW, depthimage_uint8);
    waitKey(1);
}

void ImageConverter::Cancel()
{
    this->Reset();
    ros::Duration(0.5).sleep();
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;
    n.shutdown();
    image_sub_.shutdown();
    depth_sub_.shutdown();
    destroyWindow(RGB_WINDOW);
    destroyWindow(DEPTH_WINDOW);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "KCF_Tracker");
    ros::NodeHandle nh;
    ImageConverter imageConverter(nh);
    ros::spin();
    return 0;
}

