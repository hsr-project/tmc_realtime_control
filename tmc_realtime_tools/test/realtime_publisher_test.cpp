// Copyright (C) 2026 Toyota Motor Corporation. All rights reserved.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tmc_realtime_tools/realtime_publisher.hpp>

namespace {
}  // namespace

namespace tmc_realtime_tools {

TEST(RealtimePublisherTest, BasicPublish) {
  auto pub_node = rclcpp::Node::make_shared("publisher");
  auto pub = pub_node->create_publisher<std_msgs::msg::String>("test_topic", 10);
  auto realtime_pub = std::make_shared<RealtimePublisher<std_msgs::msg::String>>(pub);

  auto sub_node = rclcpp::Node::make_shared("subscriber");
  uint32_t received_count = 0;
  auto sub = sub_node->create_subscription<std_msgs::msg::String>(
      "test_topic", 10,
      [&received_count](const std_msgs::msg::String::SharedPtr received_msg) {
        EXPECT_EQ(received_msg->data, "RealtimePublisherTest");
        received_count++;
      });

  const auto timeout = pub_node->now() + rclcpp::Duration(1, 0);
  while (pub->get_subscription_count() == 0 && pub_node->now() < timeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GT(pub->get_subscription_count(), 0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const auto start_time = sub_node->now();

  constexpr double kPublishRateHz = 100.0;
  auto rate = rclcpp::WallRate(kPublishRateHz);

  constexpr uint32_t kNumMessages = 50;
  for (int i = 0; i < kNumMessages; ++i) {
    auto msg = realtime_pub->trylock();
    if (msg) {
      msg->data = "RealtimePublisherTest";
      realtime_pub->unlockAndPublish();
    }
    rate.sleep();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(received_count, kNumMessages);

  executor.cancel();
  spin_thread.join();
}

TEST(RealtimePublisherTest, WithInitFunc) {
  auto pub_node = rclcpp::Node::make_shared("publisher");
  auto pub = pub_node->create_publisher<std_msgs::msg::String>("test_topic", 1);

  const char* const kTestData = "WithInitFunc";
  auto init_func = [kTestData](std_msgs::msg::String& msg) {
    msg.data = kTestData;
  };
  auto realtime_pub = std::make_shared<RealtimePublisher<std_msgs::msg::String>>(pub, init_func);

  auto rate = rclcpp::WallRate(1000.0);
  for (int i = 0; i < 100; ++i) {
    auto msg = realtime_pub->trylock();
    if (msg) {
      EXPECT_EQ(msg->data, kTestData);
      realtime_pub->unlockAndPublish();
    } else {
      FAIL() << "Failed to lock message buffer";
    }
    rate.sleep();
  }
}

TEST(RealtimePublisherTest, DISABLED_Performance) {
  // 1000Hzへの耐久と処理時間の確認，CIでは通らないのでDISABLED_をつける
  // 実行する場合は --gtest_also_run_disabled_tests をつけること
  auto pub_node = rclcpp::Node::make_shared("publisher");
  auto pub = pub_node->create_publisher<std_msgs::msg::String>("test_topic", 1);
  auto realtime_pub = std::make_shared<RealtimePublisher<std_msgs::msg::String>>(pub);

  auto sub_node = rclcpp::Node::make_shared("subscriber");
  uint32_t received_count = 0;
  rclcpp::Time last_received_time;
  auto sub = sub_node->create_subscription<std_msgs::msg::String>(
      "test_topic", 1,
      [&sub_node, &received_count, &last_received_time](const std_msgs::msg::String::SharedPtr received_msg) {
        EXPECT_EQ(received_msg->data, "RealtimePublisherTest");
        received_count++;
        last_received_time = sub_node->now();
      });

  const auto timeout = pub_node->now() + rclcpp::Duration(1, 0);
  while (pub->get_subscription_count() == 0 && pub_node->now() < timeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GT(pub->get_subscription_count(), 0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const auto start_time = sub_node->now();
  std::vector<int64_t> processing_times;

  constexpr double kPublishRateHz = 1000.0;
  auto rate = rclcpp::WallRate(kPublishRateHz);

  constexpr uint32_t kNumMessages = 1000;
  for (int i = 0; i < kNumMessages; ++i) {
    const auto publish_start_time = std::chrono::steady_clock::now();
    auto msg = realtime_pub->trylock();
    if (msg) {
      msg->data = "RealtimePublisherTest";
      realtime_pub->unlockAndPublish();
      const auto publish_end_time = std::chrono::steady_clock::now();
      processing_times.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
          publish_end_time - publish_start_time).count());
    }
    rate.sleep();
  }

  // ちょっとだけ待つ
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  EXPECT_EQ(processing_times.size(), kNumMessages);
  EXPECT_EQ(received_count, kNumMessages);
  // 半周期も遅れないだろう
  EXPECT_NEAR((last_received_time - start_time).seconds(), (kNumMessages - 1) / kPublishRateHz,
              1.0 / kPublishRateHz / 2.0);
  for (const auto& time : processing_times) {
    EXPECT_LT(time, 5000);
  }

  executor.cancel();
  spin_thread.join();
}

}  // namespace tmc_realtime_tools

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
