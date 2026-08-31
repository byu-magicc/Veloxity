#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/time_delay.hpp>

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

using namespace std::chrono_literals;

class TimingDriver final : public rclcpp::Node
{
public:
  TimingDriver()
  : Node("rosflight_timing_driver")
  {
    rate_hz_ = declare_parameter<double>("rate_hz", 400.0);
    duration_s_ = declare_parameter<double>("duration_s", 45.0);
    discovery_timeout_s_ = declare_parameter<double>("discovery_timeout_s", 15.0);
    settle_s_ = declare_parameter<double>("settle_s", 1.0);
    output_path_ = declare_parameter<std::string>("output_path", "publish_intervals.csv");
    rtt_output_path_ = declare_parameter<std::string>("rtt_output_path", "rtt_samples.csv");
    all_echoes_output_path_ =
      declare_parameter<std::string>("all_echoes_output_path", "all_echoes.csv");
    topic_ = declare_parameter<std::string>("topic", "/command");

    if (!(rate_hz_ > 0.0) || !(duration_s_ > 0.0) || !(settle_s_ >= 0.0)) {
      throw std::invalid_argument("rate_hz and duration_s must be positive; settle_s cannot be negative");
    }
    if (std::ceil(rate_hz_ * duration_s_) > id_scale_) {
      throw std::invalid_argument("test contains too many commands for an exact float32 sequence ID");
    }
    run_id_ = static_cast<std::uint32_t>(monotonic_ns() & 0x00FFFFFFU);
    if (run_id_ == 0) {
      run_id_ = 1;
    }

    publisher_ = create_publisher<rosflight_msgs::msg::Command>(topic_, rclcpp::QoS(1));
    delay_subscription_ = create_subscription<rosflight_msgs::msg::TimeDelay>(
      "/serial_time_delay_ns", rclcpp::QoS(10),
      [this](const rosflight_msgs::msg::TimeDelay::ConstSharedPtr message) {
        if (started_ns_ != 0) {
          record_echo(*message);
        }
      });
    discovery_started_ = std::chrono::steady_clock::now();
    discovery_timer_ = create_wall_timer(20ms, [this]() { check_discovery(); });
    RCLCPP_INFO(get_logger(), "waiting for a subscriber on %s", topic_.c_str());
  }

  ~TimingDriver() override
  {
    if (!written_) {
      write_csv();
    }
  }

  int exit_code() const { return exit_code_; }

private:
  static std::int64_t monotonic_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void check_discovery()
  {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - discovery_started_).count();
    if (publisher_->get_subscription_count() == 0) {
      if (elapsed >= discovery_timeout_s_) {
        RCLCPP_ERROR(get_logger(), "no subscriber appeared on %s within %.1f seconds",
          topic_.c_str(), discovery_timeout_s_);
        exit_code_ = 2;
        finish();
      }
      return;
    }

    discovery_timer_->cancel();
    const auto period_ns = static_cast<std::int64_t>(std::llround(1.0e9 / rate_hz_));
    publish_period_ = std::chrono::nanoseconds(period_ns);
    start_timer_ = create_wall_timer(500ms, [this]() {
      start_timer_->cancel();
      begin_publish();
    });
    RCLCPP_INFO(get_logger(), "subscriber found; test begins in 0.5 seconds");
  }

  void begin_publish()
  {
    started_ns_ = monotonic_ns();
    stop_ns_ = started_ns_ + static_cast<std::int64_t>(duration_s_ * 1.0e9);
    publish_timer_ = create_wall_timer(publish_period_, [this]() { publish_once(); });
    RCLCPP_INFO(get_logger(), "publishing at %.3f Hz for %.3f seconds", rate_hz_, duration_s_);
  }

  void publish_once()
  {
    const auto now_ns = monotonic_ns();
    if (now_ns >= stop_ns_) {
      publish_timer_->cancel();
      drain_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(settle_s_)),
        [this]() {
          drain_timer_->cancel();
          finish();
        });
      return;
    }

    rosflight_msgs::msg::Command message;
    message.header.stamp = get_clock()->now();
    message.mode = rosflight_msgs::msg::Command::MODE_PASS_THROUGH;
    message.ignore = rosflight_msgs::msg::Command::IGNORE_NONE;
    message.u.fill(0.0F);
    const auto sequence_id = static_cast<std::uint32_t>(publish_times_ns_.size() + 1);
    // Integer multiples of 2^-24 are represented exactly by float32. Keep the timing IDs in the
    // normal pass-through command range while preserving all 24 bits through MAVLink serialization.
    message.u[2] = static_cast<float>(sequence_id) / static_cast<float>(id_scale_);
    message.u[3] = static_cast<float>(run_id_) / static_cast<float>(id_scale_);
    publisher_->publish(message);
    publish_times_ns_.push_back(now_ns);
  }

  void record_echo(const rosflight_msgs::msg::TimeDelay & message)
  {
    const auto receipt_ns = monotonic_ns();
    std::string classification;

    if (message.run_id != run_id_) {
      classification = "foreign_run";
    } else if (message.sequence_id == 0 || message.sequence_id > publish_times_ns_.size()) {
      classification = "invalid_sequence";
    } else {
      const auto inserted = seen_sequences_.insert(message.sequence_id).second;
      if (!inserted) {
        classification = "duplicate";
      } else {
        classification = message.sequence_id < highest_first_sequence_
          ? "first_out_of_order" : "first";
        highest_first_sequence_ = std::max(highest_first_sequence_, message.sequence_id);
        delay_samples_.push_back(
          {message.sequence_id, receipt_ns, message.time_delay_ns});
      }
    }

    all_echoes_.push_back(
      {message.sequence_id, message.run_id, receipt_ns, message.time_delay_ns, classification});
  }

  void write_csv()
  {
    written_ = true;
    std::ofstream output(output_path_, std::ios::trunc);
    if (!output) {
      RCLCPP_ERROR(get_logger(), "cannot write %s", output_path_.c_str());
      exit_code_ = 3;
      return;
    }
    output << "index,publish_monotonic_ns,interval_ns\n";
    for (std::size_t index = 0; index < publish_times_ns_.size(); ++index) {
      const auto interval = index == 0 ? 0 : publish_times_ns_[index] - publish_times_ns_[index - 1];
      output << index << ',' << publish_times_ns_[index] << ',' << interval << '\n';
    }

    std::ofstream rtt_output(rtt_output_path_, std::ios::trunc);
    if (!rtt_output) {
      RCLCPP_ERROR(get_logger(), "cannot write %s", rtt_output_path_.c_str());
      exit_code_ = 3;
      return;
    }
    rtt_output << "index,sequence_id,receipt_monotonic_ns,rtt_ns\n";
    for (std::size_t index = 0; index < delay_samples_.size(); ++index) {
      rtt_output << index << ',' << delay_samples_[index].sequence_id << ','
                 << delay_samples_[index].receipt_ns << ','
                 << delay_samples_[index].rtt_ns << '\n';
    }

    std::ofstream all_output(all_echoes_output_path_, std::ios::trunc);
    if (!all_output) {
      RCLCPP_ERROR(get_logger(), "cannot write %s", all_echoes_output_path_.c_str());
      exit_code_ = 3;
      return;
    }
    all_output << "index,sequence_id,run_id,receipt_monotonic_ns,rtt_ns,classification\n";
    for (std::size_t index = 0; index < all_echoes_.size(); ++index) {
      const auto & echo = all_echoes_[index];
      all_output << index << ',' << echo.sequence_id << ',' << echo.run_id << ','
                 << echo.receipt_ns << ',' << echo.rtt_ns << ',' << echo.classification << '\n';
    }
  }

  void finish()
  {
    write_csv();
    RCLCPP_INFO(get_logger(),
      "published %zu commands; received %zu raw echoes and %zu unique matched echoes",
      publish_times_ns_.size(), all_echoes_.size(), delay_samples_.size());
    rclcpp::shutdown();
  }

  double rate_hz_ = 400.0;
  double duration_s_ = 45.0;
  double discovery_timeout_s_ = 15.0;
  double settle_s_ = 1.0;
  std::string output_path_;
  std::string rtt_output_path_;
  std::string all_echoes_output_path_;
  std::string topic_;
  int exit_code_ = 0;
  bool written_ = false;
  std::int64_t started_ns_ = 0;
  std::int64_t stop_ns_ = 0;
  std::chrono::nanoseconds publish_period_{0};
  std::chrono::steady_clock::time_point discovery_started_;
  std::vector<std::int64_t> publish_times_ns_;
  struct DelaySample {
    std::uint32_t sequence_id;
    std::int64_t receipt_ns;
    std::uint64_t rtt_ns;
  };
  struct EchoSample {
    std::uint32_t sequence_id;
    std::uint32_t run_id;
    std::int64_t receipt_ns;
    std::uint64_t rtt_ns;
    std::string classification;
  };
  std::vector<DelaySample> delay_samples_;
  std::vector<EchoSample> all_echoes_;
  std::unordered_set<std::uint32_t> seen_sequences_;
  std::uint32_t run_id_ = 0;
  std::uint32_t highest_first_sequence_ = 0;
  static constexpr std::uint32_t id_scale_ = 1U << 24;
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr publisher_;
  rclcpp::Subscription<rosflight_msgs::msg::TimeDelay>::SharedPtr delay_subscription_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr drain_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TimingDriver>();
    rclcpp::spin(node);
    return node->exit_code();
  } catch (const std::exception & error) {
    std::cerr << "timing_driver: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
