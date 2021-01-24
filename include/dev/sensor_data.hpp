#pragma once

#include <deque>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>

namespace dev {

/**
 * @brief 传感器数据类
 *
 */
template <class M>
class SensorData {
 public:
  typedef boost::shared_ptr<M const> MConstPtr;

  explicit SensorData(ros::NodeHandle& ros_nh, int msgs_deque_size) : nh_(ros_nh), msgs_deque_size_(msgs_deque_size) {}

  /**
   * @brief 订阅指定话题。如果首次订阅则新建订阅对象，重新订阅则使用之前的订阅对象
   * @param topic 订阅话题
   * @param queue_size
   * @param callback_queue 回调
   */
  void subscribe(const std::string& topic, uint32_t queue_size) {
    // 之前已经订阅过，需要判断是否是同一个topic
    if (sub_ && topic == cur_topic_name_) {
      // 重连即可
      sub_->subscribe();
    } else {
      sub_ = nullptr;
      // 新建订阅对象
      sub_ = boost::make_shared<message_filters::Subscriber<M>>(nh_, topic, queue_size,
                                                                ros::TransportHints().tcpNoDelay());
      // 注册回调函数
      sub_->registerCallback(
          typename message_filters::SimpleFilter<M>::Callback(boost::bind(&SensorData::callback, this, _1)));
      // 更新当前话题名称
      cur_topic_name_ = topic;
    }
  }

  /**
   * @brief 停止订阅当前话题
   */
  void unsubscribe() {
    if (sub_) {
      sub_->unsubscribe();
    }
  }

  /**
   * @brief 设置消息缓存大小
   * @param 消息缓存大小 > 0
   */
  void set_msgs_deque_size(unsigned int cache_size) {
    if (cache_size == 0) {
      return;
    }
    msgs_deque_size_ = cache_size;
  }

  /**
   * @brief 设置数据速率因子
   * @param data_rate
   */
  void set_data_rate(int data_rate) { data_rate_factor_ = data_rate; }

  MConstPtr data() {
    boost::mutex::scoped_lock lock(msgs_lock_);
    if (msgs_.empty()) {
      return nullptr;
    } else {
      return msgs_.back();
    }
  }

 private:
  /**
   * @brief 数据回调
   * @param msg
   */
  void callback(const MConstPtr& msg) {
    static int data_cnt = 0;
    if (data_cnt == data_rate_factor_) {
      data_cnt = 0;
      boost::mutex::scoped_lock lock(msgs_lock_);
      // 判断大小
      while (msgs_.size() >= msgs_deque_size_) {
        msgs_.pop_front();
      }
      // 放入队尾
      msgs_.push_back(msg);
    } else {
      data_cnt++;
    }
  }

  /// ros::NodeHandle
  ros::NodeHandle& nh_;
  /// 消息订阅对象指针
  boost::shared_ptr<message_filters::Subscriber<M>> sub_{nullptr};
  /// 当前订阅的话题
  std::string cur_topic_name_;

  /// 数据锁
  mutable boost::mutex msgs_lock_;
  /// 消息队列长度
  unsigned int msgs_deque_size_;
  /// 接收到消息队列
  std::deque<MConstPtr> msgs_;
  /// 数据速率控制
  int data_rate_factor_{0};
};

}  // namespace dev
