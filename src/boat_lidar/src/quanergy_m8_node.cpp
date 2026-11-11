// ASV ROS 2 adapter; vendor packet parsing remains in the separate Quanergy SDK.
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <quanergy/client/version.h>
#include <quanergy/client/sensor_client.h>
#include <quanergy/pipelines/sensor_pipeline.h>
#if QUANERGY_CLIENT_VERSION / 100000 != 5
#error This adapter targets Quanergy SDK major version 5.
#endif

class M8Node : public rclcpp::Node {
 public:
  M8Node() : Node("quanergy_m8") {
    const auto host = declare_parameter<std::string>("host", "");
    const auto port = declare_parameter<std::string>("port", "4141");
    const auto settings_file = declare_parameter<std::string>("settings_file", "");
    frame_ = declare_parameter<std::string>("frame_id", "lidar_frame");
    mode_ = declare_parameter<std::string>("timestamp_mode", "receive");
    clock_verified_ = declare_parameter<bool>("sensor_clock_verified", false);
    if (host.empty() || frame_.empty()) throw std::invalid_argument("host and frame_id required");
    if (mode_ != "sensor" && mode_ != "receive") throw std::invalid_argument("timestamp_mode must be sensor or receive");
    if (mode_ == "sensor" && !clock_verified_) throw std::invalid_argument("Verify sensor UTC synchronization before selecting sensor time");
    quanergy::pipeline::SensorPipelineSettings settings;
    if (!settings_file.empty()) {
      quanergy::pipeline::SettingsFileLoader loader;
      loader.loadXML(settings_file);
      settings.load(loader);
    }
    settings.host = host;
    settings.frame = frame_;
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/boat/lidar/points_raw", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>("/boat/lidar/driver_status", 10);
    pipeline_ = std::make_unique<quanergy::pipeline::SensorPipeline>(settings);
    client_ = std::make_unique<quanergy::client::SensorClient>(host, port, 100);
    packet_connection_ = client_->connect([this](const std::shared_ptr<std::vector<char>>& p) { pipeline_->slot(p); });
    cloud_connection_ = pipeline_->connect([this](const boost::shared_ptr<pcl::PointCloud<quanergy::PointXYZIR>>& cloud) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*cloud, msg);  // Retains XYZ, intensity and ring, PCL microsecond timestamp.
      msg.header.frame_id = frame_;
      if (mode_ == "receive") msg.header.stamp = now();
      const double age = (now() - rclcpp::Time(msg.header.stamp)).seconds();
      if (mode_ == "sensor" && (age < -0.1 || age > 1.0)) {
        time_error_.store(true);
        return; // Never silently relabel a foreign clock as valid UTC.
      }
      if (msg.width == 0 || msg.height == 0) return;
      cloud_pub_->publish(msg);
      time_error_.store(false);
      last_cloud_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
      ++cloud_count_;
    });
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish_status(); });
    worker_ = std::thread([this] {
      try { client_->run(); }
      catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "LiDAR client stopped: %s", e.what()); }
      failed_.store(true); // Explicit restart after failure; no silent reconnect/time reset.
    });
  }
  ~M8Node() override {
    client_->stop();
    if (worker_.joinable()) worker_.join();
    packet_connection_.disconnect();
    cloud_connection_.disconnect();
    pipeline_.reset(); // Join SDK callback worker before publishers are destroyed.
  }
 private:
  void publish_status() {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto last = last_cloud_ns_.load();
    const bool fresh = last > 0 && ns-last < 1000000000LL && !failed_.load() && !time_error_.load();
    std::ostringstream out;
    out << "{\"schema_version\":1,\"timestamp_mode\":\"" << mode_
        << "\",\"clock_verified\":" << (clock_verified_ && mode_ == "sensor" ? "true" : "false")
        << ",\"fresh\":" << (fresh ? "true" : "false")
        << ",\"client_stopped\":" << (failed_.load() ? "true" : "false")
        << ",\"timestamp_rejected\":" << (time_error_.load() ? "true" : "false")
        << ",\"cloud_count\":" << cloud_count_.load() << "}";
    std_msgs::msg::String msg; msg.data = out.str(); status_pub_->publish(msg);
  }
  std::string frame_, mode_;
  bool clock_verified_;
  std::atomic<bool> failed_{false}, time_error_{false};
  std::atomic<int64_t> last_cloud_ns_{0};
  std::atomic<uint64_t> cloud_count_{0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<quanergy::pipeline::SensorPipeline> pipeline_;
  std::unique_ptr<quanergy::client::SensorClient> client_;
  boost::signals2::connection packet_connection_, cloud_connection_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::thread worker_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<M8Node>()); }
  catch (const std::exception& e) { std::cerr << e.what() << std::endl; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
