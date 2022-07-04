/*
 *  Copyright 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_tools/network_tester/packet_sender.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "api/task_queue/queued_task.h"
#include "api/task_queue/task_queue_base.h"
#include "rtc_base/time_utils.h"
#include "rtc_tools/network_tester/config_reader.h"
#include "rtc_tools/network_tester/test_controller.h"

namespace webrtc {

namespace {

class SendPacketTask {
 public:
  explicit SendPacketTask(
      PacketSender* packet_sender,
      rtc::scoped_refptr<webrtc::PendingTaskSafetyFlag> task_safety_flag)
      : target_time_ms_(rtc::TimeMillis()),
        packet_sender_(packet_sender),
        task_safety_flag_(task_safety_flag) {}

  void operator()() {
    if (task_safety_flag_->alive() && packet_sender_->IsSending()) {
      packet_sender_->SendPacket();
      target_time_ms_ += packet_sender_->GetSendIntervalMs();
      int64_t delay_ms = std::max(static_cast<int64_t>(0),
                                  target_time_ms_ - rtc::TimeMillis());
      TaskQueueBase::Current()->PostDelayedTask(std::move(*this), delay_ms);
    }
  }

 private:
  int64_t target_time_ms_;
  PacketSender* const packet_sender_;
  rtc::scoped_refptr<webrtc::PendingTaskSafetyFlag> task_safety_flag_;
};

class UpdateTestSettingTask {
 public:
  UpdateTestSettingTask(
      PacketSender* packet_sender,
      std::unique_ptr<ConfigReader> config_reader,
      rtc::scoped_refptr<webrtc::PendingTaskSafetyFlag> task_safety_flag)
      : packet_sender_(packet_sender),
        config_reader_(std::move(config_reader)),
        task_safety_flag_(task_safety_flag) {}
  UpdateTestSettingTask(UpdateTestSettingTask&&) = default;
  UpdateTestSettingTask& operator=(UpdateTestSettingTask&&) = delete;

  void operator()() {
    if (!task_safety_flag_->alive()) {
      return;
    }
    auto config = config_reader_->GetNextConfig();
    if (config) {
      packet_sender_->UpdateTestSetting((*config).packet_size,
                                        (*config).packet_send_interval_ms);
      TaskQueueBase::Current()->PostDelayedTask(std::move(*this),
                                                (*config).execution_time_ms);
    } else {
      packet_sender_->StopSending();
    }
  }

 private:
  PacketSender* const packet_sender_;
  std::unique_ptr<ConfigReader> config_reader_;
  rtc::scoped_refptr<webrtc::PendingTaskSafetyFlag> task_safety_flag_;
};

}  // namespace

PacketSender::PacketSender(
    TestController* test_controller,
    webrtc::TaskQueueBase* worker_queue,
    rtc::scoped_refptr<webrtc::PendingTaskSafetyFlag> task_safety_flag,
    const std::string& config_file_path)
    : packet_size_(0),
      send_interval_ms_(0),
      sequence_number_(0),
      sending_(false),
      config_file_path_(config_file_path),
      test_controller_(test_controller),
      worker_queue_(worker_queue),
      task_safety_flag_(task_safety_flag) {}

PacketSender::~PacketSender() = default;

void PacketSender::StartSending() {
  worker_queue_checker_.Detach();
  worker_queue_->PostTask(ToQueuedTask(task_safety_flag_, [this]() {
    RTC_DCHECK_RUN_ON(&worker_queue_checker_);
    sending_ = true;
  }));
  worker_queue_->PostTask(UpdateTestSettingTask(
      this, std::make_unique<ConfigReader>(config_file_path_),
      task_safety_flag_));
  worker_queue_->PostTask(SendPacketTask(this, task_safety_flag_));
}

void PacketSender::StopSending() {
  RTC_DCHECK_RUN_ON(&worker_queue_checker_);
  sending_ = false;
  test_controller_->OnTestDone();
}

bool PacketSender::IsSending() const {
  RTC_DCHECK_RUN_ON(&worker_queue_checker_);
  return sending_;
}

void PacketSender::SendPacket() {
  RTC_DCHECK_RUN_ON(&worker_queue_checker_);
  NetworkTesterPacket packet;
  packet.set_type(NetworkTesterPacket::TEST_DATA);
  packet.set_sequence_number(sequence_number_++);
  packet.set_send_timestamp(rtc::TimeMicros());
  test_controller_->SendData(packet, packet_size_);
}

int64_t PacketSender::GetSendIntervalMs() const {
  RTC_DCHECK_RUN_ON(&worker_queue_checker_);
  return send_interval_ms_;
}

void PacketSender::UpdateTestSetting(size_t packet_size,
                                     int64_t send_interval_ms) {
  RTC_DCHECK_RUN_ON(&worker_queue_checker_);
  send_interval_ms_ = send_interval_ms;
  packet_size_ = packet_size;
}

}  // namespace webrtc
