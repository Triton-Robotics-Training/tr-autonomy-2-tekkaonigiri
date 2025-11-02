#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp" // publish angle
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

// member variables at the bottom instead of in a header file

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
        "random_image", 10);
    subscription_ =
        this->create_subscription<sensor_msgs::msg::Image>(
        "/robotcam", 10,
	    std::bind(&MinimalImagePublisher::callback_function, this, std::placeholders::_1)
        );

    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalImagePublisher::timer_callback, this));
    
    publish_angle_ = this->create_publisher<std_msgs::msg::Float32>("desired_angle", 10);
        // assign publisher obj to member variable
        // desired_angle topic name
  }

private:

  void callback_function(
        const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    // convert ros image to opencv mat // declare a pointer to CvImage object
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            // bgr8 common color for encoding opencv
            // cv_ptr will evnetually hold image that can be manipulated
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
            // log error msg to ROS2 console // e.what() exactly what went wrong
            // immediately reutnr to prevent crash
    }



    // -- find target position --

    // convert BGR to HSV (hue saturation value)
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // find specific value for target --> red // filter mask
    cv::Scalar lower_red(0, 100, 100);
    cv::Scalar upper_red(0, 255, 255);
        // cv::Scalar holds hsv values
    
    cv::Mat color_mask;
    cv::inRange(hsv_image, lower_red, upper_red, color_mask);
        // cv::inRange creates binary mask (black n white)

    // find target cell -- calculate image moments of mask to find center
    cv::Moments m = cv::moments(color_mask, false);
    double x_cord = 0.0;
    double y_cord = 0.0;
        // center of mass

    // if target is detected --> check if total area isn't just noise
        // m.m00 total area
            // m = moment value of image contour
            // 00 = zeroth moment = xy
                // 10 = 1st moment along x, 01 = 1st moment along y
        // 1000 make sure no noise // threshold/filter
    if (m.m00 > 1000) {
        x_cord = m.m10 / m.m00; // centroid x cord
        y_cord = m.m01 / m.m00; // centroid y cord

        // debugger
        RCLCPP_INFO(this->get_logger(), "Target found at X: %.2f, Y: %.2f", x_cord, y_cord);
    } else {
        RCLCPP_INFO(this->get_logger(), "Target not found. Skip angle publish.");
        return;
            // no target found, dont put out anything
    }


    // -- publish angle /desired_angle --
    const double IMAGE_CENTER_X = 320.0;
        // assume 640 pixel width
        // defines ideal center of image
            // if target at X position -> robot pointed correctly
    const double PIXELS_TO_ANGLE_RATIO = 0.005;
        // scale/gain factor
        // used to tune sensitivity
            // larger # = more aggressive response to small pixel offset
    
    double pixel_offset = x_cord - IMAGE_CENTER_X;
        // calc diff btwn target current position + center
    double desired_angle = pixel_offset * PIXELS_TO_ANGLE_RATIO;
        // convert pixel diff into physical angle

    // create float32 msg (unique pointer for new msg)
    auto angle_msg = std::make_unique<std_msgs::msg::Float32>();
    angle_msg->data = static_cast<float>(desired_angle);

    // publish angle
    publish_angle_->publish(std::move(angle_msg));
    RCLCPP_INFO(this->get_logger(), "Published angle: %.2f", desired_angle);
        // debugger
  }

  void timer_callback() {
    // Create a new 640x480 image
    cv::Mat my_image(cv::Size(640, 480), CV_8UC3);

    // Generate an image where each pixel is a random color
    cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image)
               .toImageMsg();

    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++;
  }


  // member variables

  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publish_angle_;
        // type for ros2 publisher smart pointer
        // publish_angle_ object


};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();

  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}