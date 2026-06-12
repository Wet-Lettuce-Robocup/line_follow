#include "line_follow/navigation.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <functional>
#include <unordered_map>
#include <cv_bridge/cv_bridge.hpp>

using std::placeholders::_1;

NavigationNode::NavigationNode()
: rclcpp_lifecycle::LifecycleNode("navigation")
{
  this->declare_parameter<std::string>("navigation_type", "simple");

  std::string nav_type_str = this->get_parameter("navigation_type").as_string();

  static const std::unordered_map<std::string, NavigationType> nav_type_map = {
    {"simple", NavigationType::SIMPLE},
    {"advanced", NavigationType::ADVANCED}
  };

  auto it = nav_type_map.find(nav_type_str);

  if (it != nav_type_map.end()) {
    this->navigationType = it->second;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown navigation type! Defaulting to simple.");
    this->navigationType = NavigationType::SIMPLE;
  }
}

CallbackReturn NavigationNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");
  this->errorPub = this->create_publisher<std_msgs::msg::Float64>("line_error", 10);
  this->imageSub = this->create_subscription<sensor_msgs::msg::Image>("camera_front/image_raw", 10,
    std::bind(&NavigationNode::imageCallback, this, _1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn NavigationNode::on_activate(const rclcpp_lifecycle::State & state)
{
  this->errorPub->on_activate();

  return rclcpp_lifecycle::LifecycleNode::on_activate(state);
}

CallbackReturn NavigationNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  this->errorPub->on_deactivate();

  return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

CallbackReturn NavigationNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  this->errorPub.reset();
  this->imageSub.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn NavigationNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void NavigationNode::imageCallback(sensor_msgs::msg::Image::SharedPtr msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }

  cv::Mat frame = cv_ptr->image;

  switch (this->navigationType) {
    case NavigationType::SIMPLE:
      this->simpleNavigation(frame);
      break;
    case NavigationType::ADVANCED:
      this->advancedNavigation(frame);
      break;
  }
}

void NavigationNode::simpleNavigation(cv::Mat & frame)
{
  cv::Mat green = this->getGreen(frame);

  cv::Mat line;
  cv::inRange(frame, cv::Scalar(0, 0, 0), cv::Scalar(60, 60, 60), line);

  float greenWeight = 2.0;
  cv::Mat weightedCombined;
  cv::addWeighted(green, greenWeight, line, 1.0, 0, weightedCombined);

  cv::Mat grayscale;
  cv::cvtColor(weightedCombined, grayscale, cv::COLOR_BGR2GRAY);

  cv::Mat combined;
  cv::threshold(weightedCombined, combined, 0, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours(combined, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  double linePos = 0;

  if (contours.empty()) {
    this->publishError(0);
    return;
  }

  int xPos = 0;
  int yPos = 0;
  int xPosGreen = 0;
  int yPosGreen = 0;

  // Find the largest contour by area
  auto largestContourLine = std::max_element(contours.begin(), contours.end(),
      [](const std::vector<cv::Point> & c1, const std::vector<cv::Point> & c2) {
        return cv::contourArea(c1) < cv::contourArea(c2);
            });

        // Calculate moments and center of mass for the line
  cv::Moments m = cv::moments(*largestContourLine);
  if (m.m00 == 0) {
    this->publishError(linePos / 200);
    return;
  }

  xPos = static_cast<int>(m.m10 / m.m00);
  yPos = static_cast<int>(m.m01 / m.m00);

        // Process Green ROI
  std::vector<std::vector<cv::Point>> greenContours;
  cv::findContours(green, greenContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point> largestContourGreen;
  bool greenFound = false;

  if (!greenContours.empty()) {
    auto largestGreenIt = std::max_element(greenContours.begin(), greenContours.end(),
        [](const std::vector<cv::Point> & c1, const std::vector<cv::Point> & c2) {
          return cv::contourArea(c1) < cv::contourArea(c2);
                });

    largestContourGreen = *largestGreenIt;
    greenFound = true;

    cv::Moments mGreen = cv::moments(largestContourGreen);
    if (mGreen.m00 != 0) {
      xPosGreen = static_cast<int>(mGreen.m10 / mGreen.m00);
      yPosGreen = static_cast<int>(mGreen.m01 / mGreen.m00);
    } else {
      xPosGreen = 0;
      yPosGreen = 0;
    }
  } else {
    xPosGreen = 0;
    yPosGreen = 0;
  }

        // Average green and line Center of Mass
  if (greenFound && cv::contourArea(largestContourGreen) > 1000) {
    xPos = static_cast<int>((xPos + xPosGreen) / 2.0);
    yPos = static_cast<int>((yPos + yPosGreen) / 2.0);
  } else {
    xPosGreen = 0;
    yPosGreen = 0;
  }

  if (yPos > 300) { // if line is behind robot (adjusted for 360 height)
    if (xPos > 240) { // if line is to the right
      linePos = 100;
    } else if (xPos < 240) { // if line is to the left
      linePos = -100;
    } else {
      std::cout << "line is really cooked its fully behind" << std::endl;
      linePos = 0;
    }
  } else if (yPos == 360) {
    linePos = 0;
  } else {
    // Math translation: math.degrees(x) in C++ is x * (180.0 / M_PI)
    double radians = std::atan((xPos - 240.0) / (360.0 - yPos));
    linePos = radians * (180.0 / 3.14159265358979323846);
  }

  this->publishError(linePos / 200);
}

void NavigationNode::advancedNavigation(cv::Mat & frame)
{
  cv::Mat processed = this->processImage(frame);
}

cv::Mat NavigationNode::processImage(cv::Mat & image)
{
  cv::Mat resized;
  cv::Size dsize(200, 100);
  cv::resize(image, resized, dsize);

  cv::Mat gray, binary;
  cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);

  cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
    11, 2);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

  cv::Mat opened_image;
  cv::morphologyEx(binary, opened_image, cv::MORPH_OPEN, kernel);

  cv::Mat closed_image;
  cv::morphologyEx(opened_image, closed_image, cv::MORPH_CLOSE, kernel);

  cv::Mat skeleton;
  cv::ximgproc::thinning(closed_image, skeleton,
                         cv::ximgproc::THINNING_GUOHALL);

  int rows = skeleton.rows;
  int cols = skeleton.cols;

  cv::Rect top_border_roi(0, 0, cols, 1);
  skeleton(top_border_roi).setTo(0);

  // Set bottom border
  cv::Rect bottom_border_roi(0, rows - 1, cols, 1);
  skeleton(bottom_border_roi).setTo(0);

  // Set left border (excluding corners already set)
  cv::Rect left_border_roi(0, 1, 1, rows - 2);
  skeleton(left_border_roi).setTo(0);

  // Set right border (excluding corners already set)
  cv::Rect right_border_roi(cols - 1, 1, 1, rows - 2);
  skeleton(right_border_roi).setTo(0);

  return skeleton;
}

cv::Mat getGreen(cv::Mat & image)
{
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Scalar lower_green(35, 40, 40);
  cv::Scalar upper_green(85, 255, 255);

  cv::Mat mask;
  cv::inRange(hsv, lower_green, upper_green, mask);

  return mask;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
