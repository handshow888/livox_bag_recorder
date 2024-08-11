#include <chrono>
#include <iomanip>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writer_interfaces/base_writer_interface.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

class BagRecorder : public rclcpp::Node
{
public:
    BagRecorder() : Node("bag_recorder")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();

        sub_pointcloud2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar/pointcloud2", qos, std::bind(&BagRecorder::pointcloud2_callback, this, std::placeholders::_1));
        sub_customMsg = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", qos, std::bind(&BagRecorder::custom_callback, this, std::placeholders::_1));
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", qos, std::bind(&BagRecorder::imu_callback, this, std::placeholders::_1));

        // std::string timestamp = std::to_string(this->get_clock()->now().seconds());
        // std::string bag_file_dir = "rosbag2_" + timestamp;
        std::string formattedTimestamp = getFormattedTimestamp();
        std::string bag_file_dir = std::string(ROOT_DIR) + "bag_files/" + "rosbag2_" + formattedTimestamp;
        RCLCPP_INFO(this->get_logger(), "bag saving to folder: %s", bag_file_dir.c_str());

        writer_ = std::make_shared<rosbag2_cpp::Writer>();
        writer_->open(bag_file_dir);
    }

    void pointcloud2_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        writer_->write(*msg, "/livox/lidar/pointcloud2", msg->header.stamp);
    }
    void custom_callback(livox_ros_driver2::msg::CustomMsg::SharedPtr msg) const
    {
        writer_->write(*msg, "/livox/lidar", msg->header.stamp);
    }
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) const
    {
        writer_->write(*msg, "/livox/imu", msg->header.stamp);
    }

    // 获取当前时间并格式化为 "YYYYMMDDHHMM"
    std::string getFormattedTimestamp()
    {
        // 获取当前时间
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        // 转换为 tm 结构体
        std::tm *tm_time = std::localtime(&now_time);
        // 使用 stringstream 进行格式化
        std::stringstream ss;
        ss << std::put_time(tm_time, "%Y-%m-%d-%H-%M-%S");
        // 返回格式化后的字符串
        return ss.str();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud2;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_customMsg;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    std::shared_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // 使用多线程执行器
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<BagRecorder>();
    // 将节点添加到执行器中
    executor->add_node(node);
    // 运行执行器
    executor->spin();
    rclcpp::shutdown();
    return 0;
}