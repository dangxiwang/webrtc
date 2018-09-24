/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "logging/rtc_event_log/rtc_event_processor.h"

#include <initializer_list>
#include <numeric>

#include "absl/memory/memory.h"
#include "logging/rtc_event_log/rtc_event_log_parser_new.h"
#include "rtc_base/checks.h"
#include "rtc_base/random.h"
#include "test/gtest.h"

namespace webrtc {

namespace {
std::vector<LoggedStartEvent> CreateEventList(
    std::initializer_list<int64_t> timestamp_list) {
  std::vector<LoggedStartEvent> v;
  for (int64_t timestamp_ms : timestamp_list) {
    v.emplace_back(timestamp_ms * 1000);  // Convert ms to us.
  }
  return v;
}

using OrderedEventView = ProcessableEventList<LoggedStartEvent>;

std::vector<std::vector<LoggedStartEvent>>
CreateRandomEventLists(size_t num_lists, size_t num_elements, uint64_t seed) {
  Random prng(seed);
  std::vector<std::vector<LoggedStartEvent>> lists(num_lists);
  for (size_t elem = 0; elem < num_elements; elem++) {
    uint32_t i = prng.Rand(0u, num_lists - 1);
    int64_t timestamp_ms = elem;
    lists[i].emplace_back(timestamp_ms * 1000);
  }
  return lists;
}
}  // namespace

TEST(RtcEventProcessor, NoList) {
  RtcEventProcessor processor;
  processor.ProcessEventsInOrder();  // Don't crash but do nothing.
}

TEST(RtcEventProcessor, EmptyList) {
  auto not_called = [](LoggedStartEvent /*elem*/) { EXPECT_TRUE(false); };
  std::vector<LoggedStartEvent> events;
  RtcEventProcessor processor;

  processor.AddEvents(absl::make_unique<OrderedEventView>(
      events.begin(), events.end(), not_called));
  processor.ProcessEventsInOrder();  // Don't crash but do nothing.
}

TEST(RtcEventProcessor, OneList) {
  std::vector<LoggedStartEvent> result;
  auto f = [&result](LoggedStartEvent elem) { result.push_back(elem); };

  std::vector<LoggedStartEvent> events(CreateEventList({1, 2, 3, 4}));
  RtcEventProcessor processor;
  processor.AddEvents(
      absl::make_unique<OrderedEventView>(events.begin(), events.end(), f));
  processor.ProcessEventsInOrder();

  std::vector<int64_t> expected_results{1, 2, 3, 4};
  ASSERT_EQ(result.size(), expected_results.size());
  for (size_t i = 0; i < expected_results.size(); i++) {
    EXPECT_EQ(result[i].log_time_ms(), expected_results[i]);
  }
}

TEST(RtcEventProcessor, MergeTwoLists) {
  std::vector<LoggedStartEvent> result;
  auto f = [&result](LoggedStartEvent elem) { result.push_back(elem); };

  std::vector<LoggedStartEvent> events1(CreateEventList({1, 2, 4, 7, 8, 9}));
  std::vector<LoggedStartEvent> events2(CreateEventList({3, 5, 6, 10}));
  RtcEventProcessor processor;
  processor.AddEvents(
      absl::make_unique<OrderedEventView>(events1.begin(), events1.end(), f));
  processor.AddEvents(
      absl::make_unique<OrderedEventView>(events2.begin(), events2.end(), f));
  processor.ProcessEventsInOrder();

  std::vector<int64_t> expected_results{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  ASSERT_EQ(result.size(), expected_results.size());
  for (size_t i = 0; i < expected_results.size(); i++) {
    EXPECT_EQ(result[i].log_time_ms(), expected_results[i]);
  }
}

TEST(RtcEventProcessor, MergeTwoListsWithDuplicatedElements) {
  std::vector<LoggedStartEvent> result;
  auto f = [&result](LoggedStartEvent elem) { result.push_back(elem); };

  std::vector<LoggedStartEvent> events1(CreateEventList({1, 2, 2, 3, 5, 5}));
  std::vector<LoggedStartEvent> events2(CreateEventList({1, 3, 4, 4}));
  RtcEventProcessor processor;
  processor.AddEvents(
      absl::make_unique<OrderedEventView>(events1.begin(), events1.end(), f));
  processor.AddEvents(
      absl::make_unique<OrderedEventView>(events2.begin(), events2.end(), f));
  processor.ProcessEventsInOrder();

  std::vector<int64_t> expected_results{1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
  ASSERT_EQ(result.size(), expected_results.size());
  for (size_t i = 0; i < expected_results.size(); i++) {
    EXPECT_EQ(result[i].log_time_ms(), expected_results[i]);
  }
}

TEST(RtcEventProcessor, MergeManyLists) {
  std::vector<LoggedStartEvent> result;
  auto f = [&result](LoggedStartEvent elem) { result.push_back(elem); };

  constexpr size_t kNumLists = 5;
  constexpr size_t kNumElems = 30;
  constexpr uint64_t kSeed = 0xF3C6B91F;
  std::vector<std::vector<LoggedStartEvent>> lists(
      CreateRandomEventLists(kNumLists, kNumElems, kSeed));
  RTC_DCHECK_EQ(lists.size(), kNumLists);
  RtcEventProcessor processor;
  for (const auto& list : lists) {
    processor.AddEvents(
        absl::make_unique<OrderedEventView>(list.begin(), list.end(), f));
  }
  processor.ProcessEventsInOrder();

  std::vector<int64_t> expected_results(kNumElems);
  std::iota(expected_results.begin(), expected_results.end(), 0);
  ASSERT_EQ(result.size(), expected_results.size());
  for (size_t i = 0; i < expected_results.size(); i++) {
    EXPECT_EQ(result[i].log_time_ms(), expected_results[i]);
  }
}

TEST(RtcEventProcessor, DifferentTypes) {
  std::vector<int64_t> result;
  auto f1 = [&result](LoggedStartEvent elem) {
    result.push_back(elem.log_time_ms());
  };
  auto f2 = [&result](LoggedStopEvent elem) {
    result.push_back(elem.log_time_ms());
  };

  std::vector<LoggedStartEvent> events1{LoggedStartEvent(2000)};
  std::vector<LoggedStopEvent> events2{LoggedStopEvent(1000)};
  RtcEventProcessor processor;
  processor.AddEvents(absl::make_unique<ProcessableEventList<LoggedStartEvent>>(
      events1.begin(), events1.end(), f1));
  processor.AddEvents(absl::make_unique<ProcessableEventList<LoggedStopEvent>>(
      events2.begin(), events2.end(), f2));
  processor.ProcessEventsInOrder();

  std::vector<int64_t> expected_results{1, 2};
  ASSERT_EQ(result.size(), expected_results.size());
  for (size_t i = 0; i < expected_results.size(); i++) {
    EXPECT_EQ(result[i], expected_results[i]);
  }
}

}  // namespace webrtc
