#include <librealsense2/rs.hpp>
#include <librealsense2-net/rs_net.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

class RealSensePublisher : public rclcpp::Node
{
public:
    RealSensePublisher() : Node("realsense_publisher")
    {
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/camera/aligned_depth_to_color/image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10);

        this->declare_parameter("ip_address", "192.168.1.189");
        this->declare_parameter("use_network", true);
        this->declare_parameter("color_width", 640);
        this->declare_parameter("color_height", 480);
        this->declare_parameter("depth_width", 640);
        this->declare_parameter("depth_height", 480);
        this->declare_parameter("framerate", 30);

        if (!initialize_camera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&RealSensePublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "RealSense publisher node started");
    }

    ~RealSensePublisher()
    {
        cleanup();
    }

private:
    bool initialize_camera()
    {
        try {
            std::string ip_address = this->get_parameter("ip_address").as_string();
            bool use_network = this->get_parameter("use_network").as_bool();
            int color_width = this->get_parameter("color_width").as_int();
            int color_height = this->get_parameter("color_height").as_int();
            int depth_width = this->get_parameter("depth_width").as_int();
            int depth_height = this->get_parameter("depth_height").as_int();
            int framerate = this->get_parameter("framerate").as_int();

            ctx_ = std::make_shared<rs2::context>();

            if (use_network) {
                net_dev_ = std::make_shared<rs2::net_device>(ip_address);
                net_dev_->add_to(*ctx_);
                RCLCPP_INFO(this->get_logger(), "Using network device at %s", ip_address.c_str());
            }

            pipe_ = std::make_shared<rs2::pipeline>(*ctx_);

            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_BGR8, framerate);
            cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, framerate);

            profile_ = pipe_->start(cfg);
            align_ = std::make_shared<rs2::align>(RS2_STREAM_COLOR);

            extract_intrinsics();

            RCLCPP_INFO(this->get_logger(), "Camera initialized successfully");
            return true;

        } catch (const rs2::error &e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
            return false;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
            return false;
        }
    }

    void extract_intrinsics()
    {
        try {
            auto color_stream = profile_.get_stream(RS2_STREAM_COLOR);
            auto color_intrinsics = color_stream.as<rs2::video_stream_profile>().get_intrinsics();

            fx_ = color_intrinsics.fx;
            fy_ = color_intrinsics.fy;
            ppx_ = color_intrinsics.ppx;
            ppy_ = color_intrinsics.ppy;
            width_ = color_intrinsics.width;
            height_ = color_intrinsics.height;

            RCLCPP_INFO(this->get_logger(), "Intrinsics - fx: %.2f, fy: %.2f, ppx: %.2f, ppy: %.2f", 
                       fx_, fy_, ppx_, ppy_);

        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Could not extract intrinsics: %s", e.what());
            fx_ = fy_ = 500.0f;
            ppx_ = width_ / 2.0f;
            ppy_ = height_ / 2.0f;
        }
    }

    void timer_callback()
    {
        try {
            rs2::frameset frames = pipe_->wait_for_frames(1000);
            
            if (!frames) {
                return;
            }

            frames = align_->process(frames);

            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();

            if (!color_frame || !depth_frame) {
                return;
            }

            auto now = this->now();

            publish_color_frame(color_frame, now);
            publish_depth_frame(depth_frame, now);
            publish_camera_info(now);

        } catch (const rs2::error &e) {
            RCLCPP_WARN(this->get_logger(), "Frame capture error: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Timer callback error: %s", e.what());
        }
    }

    void publish_color_frame(const rs2::video_frame& frame, const rclcpp::Time& timestamp)
    {
        cv::Mat color_mat(cv::Size(frame.get_width(), frame.get_height()), 
                         CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_mat).toImageMsg();
        msg->header.stamp = timestamp;
        msg->header.frame_id = "camera_color_optical_frame";

        color_pub_->publish(*msg);
    }

    void publish_depth_frame(const rs2::depth_frame& frame, const rclcpp::Time& timestamp)
    {
        cv::Mat depth_mat(cv::Size(frame.get_width(), frame.get_height()), 
                         CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_mat).toImageMsg();
        msg->header.stamp = timestamp;
        msg->header.frame_id = "camera_depth_optical_frame";

        depth_pub_->publish(*msg);
    }

    void publish_camera_info(const rclcpp::Time& timestamp)
    {
        auto msg = sensor_msgs::msg::CameraInfo();
        
        msg.header.stamp = timestamp;
        msg.header.frame_id = "camera_color_optical_frame";
        
        msg.width = width_;
        msg.height = height_;
        msg.distortion_model = "plumb_bob";
        
        msg.k.fill(0.0);
        msg.k[0] = fx_;
        msg.k[2] = ppx_;
        msg.k[4] = fy_;
        msg.k[5] = ppy_;
        msg.k[8] = 1.0;
        
        msg.r.fill(0.0);
        msg.r[0] = msg.r[4] = msg.r[8] = 1.0;
        
        msg.p.fill(0.0);
        msg.p[0] = fx_;
        msg.p[2] = ppx_;
        msg.p[5] = fy_;
        msg.p[6] = ppy_;
        msg.p[10] = 1.0;
        
        msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        
        camera_info_pub_->publish(msg);
    }

    void cleanup()
    {
        try {
            if (pipe_) {
                pipe_->stop();
            }
        } catch (const std::exception &e) {
            RCLCPP_DEBUG(this->get_logger(), "Cleanup error: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<rs2::context> ctx_;
    std::shared_ptr<rs2::net_device> net_dev_;
    std::shared_ptr<rs2::pipeline> pipe_;
    rs2::pipeline_profile profile_;
    std::shared_ptr<rs2::align> align_;

    float fx_, fy_, ppx_, ppy_;
    int width_, height_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RealSensePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Node error: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
