/**
 * @file KCF_Tracker.cpp
 * @brief ROS KCF 目标跟踪节点 — 鼠标框选 ROI，KCF 实时跟踪，发布目标像素位置与深度。
 *
 * 订阅彩色图像 /camera/color/image_raw 与深度图像 /camera/depth/image_raw，
 * 用户通过鼠标在画面中框选 ROI，节点初始化 KCF 跟踪器并对目标进行实时跟踪。
 * 跟踪结果以像素误差 (x, y) 与深度距离 (z) 的形式发布。
 *
 * 话题：
 *   订阅 — /camera/color/image_raw (sensor_msgs::Image, BGR8)
 *          /camera/depth/image_raw (sensor_msgs::Image, TYPE_32FC1)
 *   发布 — /tracker/target_position (geometry_msgs::Point)
 *          /tracker/is_tracking    (std_msgs::Bool)
 *
 * 键盘控制：
 *   r — 重置 ROI 选择
 *   q / ESC — 退出节点
 *
 * 使用：
 *   rosrun uav_tracker KCFTracker_node
 *   roslaunch uav_tracker KCFTracker.launch
 */

#include "KCF_Tracker.h"

// ============================================================================
// 全局变量 — 鼠标回调与 ROI 状态
// ============================================================================

Rect selectRect;                    ///< 鼠标拖拽框选区域
Point origin;                       ///< 鼠标按下起始点 [pixel]
Rect result;                        ///< KCF 跟踪结果矩形
bool select_flag = false;           ///< 鼠标左键按下标志
bool bRenewROI = false;             ///< 新 ROI 已选择，触发 KCF 重新初始化
bool bBeginKCF = false;             ///< KCF 跟踪已启动
Mat rgbimage;                       ///< 当前彩色图像帧
Mat depthimage;                     ///< 当前深度图像帧
float minDist = 0;                  ///< 跟踪目标最小深度距离 [m]
const int &ACTION_ESC = 27;         ///< ESC 键 ASCII 码

// ============================================================================
// 鼠标回调
// ============================================================================

/**
 * @brief 鼠标事件回调 — 拖拽框选目标 ROI。
 *
 * 左键按下记录起始点，拖动过程中实时更新选框，左键释放时设置 bRenewROI
 * 标志以触发 KCF 跟踪器重新初始化。
 *
 * @param event  OpenCV 鼠标事件类型 (EVENT_LBUTTONDOWN / EVENT_LBUTTONUP)
 * @param x      鼠标当前 x 坐标 [pixel]
 * @param y      鼠标当前 y 坐标 [pixel]
 */
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

// ============================================================================
// ImageConverter 实现
// ============================================================================

/**
 * @brief 构造函数 — 初始化 KCF 跟踪器、订阅话题、发布话题、创建 GUI 窗口。
 *
 * 算法流程：
 *   1. 构造 KCFTracker (HOG + 固定窗口 + 多尺度 + LAB)
 *   2. 订阅彩色图像 /camera/color/image_raw 与深度图像 /camera/depth/image_raw
 *   3. 注册目标位置与跟踪状态发布者
 *   4. 从参数服务器加载深度限制与窗口显示配置
 *   5. 创建 OpenCV GUI 窗口
 *
 * @param n ROS 私有节点句柄 (~)
 */
ImageConverter::ImageConverter(ros::NodeHandle &n)
{
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
    // Subscrive to input video feed and publish output video feed
    image_sub_ = n.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
    depth_sub_ = n.subscribe("/camera/depth/image_raw", 1, &ImageConverter::depthCb, this);

    target_pub_ = n.advertise<geometry_msgs::Point>("/tracker/target_position", 1);
    status_pub_ = n.advertise<std_msgs::Bool>("/tracker/is_tracking", 1);

    n.param("limit_max_dist", LIMIT_MAX_DIST, 10.0f);
    n.param("limit_max_dist", LIMIT_MIN_DIST, 0.3f);
    n.param("show_depth_window", SHOW_DEPTH_WINDOW, false);

    namedWindow(RGB_WINDOW);
    if(SHOW_DEPTH_WINDOW) namedWindow(DEPTH_WINDOW);
}

ImageConverter::~ImageConverter()
{
    n.shutdown();
    image_sub_.shutdown();
    depth_sub_.shutdown();
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;
    destroyWindow(RGB_WINDOW);
    if(SHOW_DEPTH_WINDOW) destroyWindow(DEPTH_WINDOW);
}

/**
 * @brief 重置跟踪状态 — 清空 ROI、停止 KCF、关闭深度获取。
 */
void ImageConverter::Reset() {
    bRenewROI = false;
    bBeginKCF = false;
    selectRect.x = 0;
    selectRect.y = 0;
    selectRect.width = 0;
    selectRect.height = 0;
    enable_get_depth = false;
}

/**
 * @brief 彩色图像回调 — 核心跟踪流水线。
 *
 * 算法流程：
 *   1. cv_bridge 将 ROS Image 转为 OpenCV BGR8 Mat
 *   2. 绑定鼠标回调，等待用户框选 ROI
 *   3. 若 bRenewROI 置位 → 用 selectRect 初始化 KCF 跟踪器，bBeginKCF = true
 *   4. 若 bBeginKCF 置位 → 调用 tracker.update() 更新目标位置
 *   5. 在画面上绘制跟踪框 (黄色矩形)、中心点 (红色圆) 与坐标文字
 *   6. 计算像素误差 (相对图像中心) 与深度距离
 *   7. 发布 /tracker/target_position (pixel_err_x, pixel_err_y, depth_z)
 *   8. 发布 /tracker/is_tracking (深度有效则为 true)
 *   9. 显示画面，检测键盘输入 (r=重置, q/ESC=退出)
 *
 * @param msg 彩色图像消息 (BGR8 编码)
 */
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

        target_pos_msg.x = error_x;   // 像素误差 [pixel]
        target_pos_msg.y = error_y;   // 像素误差 [pixel]
        target_pos_msg.z = minDist;   // 深度距离 [m]

        // 简单的可信度检查：如果深度无限远或为0，可能丢失
        if(minDist != std::numeric_limits<double>::quiet_NaN() && minDist != std::numeric_limits<double>::infinity())
            is_tracking_msg.data = true;
        else
            is_tracking_msg.data = false; // 深度异常视为丢失
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

// ============================================================================
// 深度回调
// ============================================================================

/**
 * @brief 深度图像回调 — 提取跟踪框中心区域的深度值。
 *
 * 算法流程：
 *   1. cv_bridge 将 ROS Image 转为 OpenCV 32FC1 Mat (单位: mm → /1000 转 m)
 *   2. 若 enable_get_depth 置位 → 在 result 中心 5 点采样深度
 *   3. 过滤超出 LIMIT_MIN_DIST ~ LIMIT_MAX_DIST 范围的异常值
 *   4. 取有效采样点均值作为 minDist（目标深度距离）
 *   5. 若 SHOW_DEPTH_WINDOW 为 true → 归一化并显示深度伪彩图
 *
 * @param msg 深度图像消息 (TYPE_32FC1 编码, 单位为 mm)
 */
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
            if (dist_val[i] > LIMIT_MIN_DIST && dist_val[i] < LIMIT_MAX_DIST) minDist += dist_val[i];
            else num_depth_points--;
        }
        minDist /= num_depth_points;
    }

    if(SHOW_DEPTH_WINDOW)
    {
        Mat depthimage_uint8;
        normalize(depthimage, depthimage_uint8, 0, 255, NORM_MINMAX);
        convertScaleAbs(depthimage_uint8, depthimage_uint8);
        imshow(DEPTH_WINDOW, depthimage_uint8);
    }

    waitKey(1);
}

/**
 * @brief 取消跟踪 — 重置状态，关闭 GUI 窗口，关闭 ROS 话题。
 */
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
    if(SHOW_DEPTH_WINDOW) destroyWindow(DEPTH_WINDOW);
}

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief ROS 节点入口 — 初始化 "KCF_Tracker" 节点，启动 ImageConverter。
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表
 * @return int 退出码 (0 = 正常)
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "KCF_Tracker");
    ros::NodeHandle nh("~");
    ImageConverter imageConverter(nh);
    ros::spin();
    return 0;
}
