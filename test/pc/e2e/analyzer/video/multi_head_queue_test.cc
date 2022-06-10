/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "test/pc/e2e/analyzer/video/multi_head_queue.h"
#include "absl/types/optional.h"
#include "test/gtest.h"

namespace webrtc {
namespace {

TEST(MultiHeadQueueTest, EmptyQueueEmptyForAllHeads) {
  MultiHeadQueue<int> queue = MultiHeadQueue<int>(10);
  EXPECT_EQ(queue.size(), 0lu);
  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(queue.IsEmpty(i));
    EXPECT_EQ(queue.size(i), 0lu);
    EXPECT_FALSE(queue.PopFront(i).has_value());
    EXPECT_FALSE(queue.Front(i).has_value());
  }
}

TEST(MultiHeadQueueTest, SizeIsEqualForAllHeadsAfterAddOnly) {
  MultiHeadQueue<int> queue = MultiHeadQueue<int>(10);
  queue.PushBack(1);
  queue.PushBack(2);
  queue.PushBack(3);
  EXPECT_EQ(queue.size(), 3lu);
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(queue.IsEmpty(i));
    EXPECT_EQ(queue.size(i), 3lu);
  }
}

TEST(MultiHeadQueueTest, SizeIsCorrectAfterRemoveFromOnlyOneHead) {
  MultiHeadQueue<int> queue = MultiHeadQueue<int>(10);
  for (int i = 0; i < 5; ++i) {
    queue.PushBack(i);
  }
  EXPECT_EQ(queue.size(), 5lu);
  // Removing elements from queue #0
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(queue.size(0), static_cast<size_t>(5 - i));
    EXPECT_EQ(queue.PopFront(0), absl::optional<int>(i));
  }
  EXPECT_EQ(queue.size(0), 0lu);
  EXPECT_TRUE(queue.IsEmpty(0));
}

TEST(MultiHeadQueueTest, SingleHeadOneAddOneRemove) {
  MultiHeadQueue<int> queue = MultiHeadQueue<int>(1);
  queue.PushBack(1);
  EXPECT_EQ(queue.size(), 1lu);
  EXPECT_TRUE(queue.Front(0).has_value());
  EXPECT_EQ(queue.Front(0).value(), 1);
  absl::optional<int> value = queue.PopFront(0);
  EXPECT_TRUE(value.has_value());
  EXPECT_EQ(value.value(), 1);
  EXPECT_EQ(queue.size(), 0lu);
  EXPECT_TRUE(queue.IsEmpty(0));
}

TEST(MultiHeadQueueTest, SingleHead) {
  MultiHeadQueue<size_t> queue = MultiHeadQueue<size_t>(1);
  for (size_t i = 0; i < 10; ++i) {
    queue.PushBack(i);
    EXPECT_EQ(queue.size(), i + 1);
  }
  for (size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(queue.Front(0), absl::optional<size_t>(i));
    EXPECT_EQ(queue.PopFront(0), absl::optional<size_t>(i));
    EXPECT_EQ(queue.size(), 10 - i - 1);
  }
}

TEST(MultiHeadQueueTest, ThreeHeadsAddAllRemoveAllPerHead) {
  MultiHeadQueue<size_t> queue = MultiHeadQueue<size_t>(3);
  for (size_t i = 0; i < 10; ++i) {
    queue.PushBack(i);
    EXPECT_EQ(queue.size(), i + 1);
  }
  for (size_t i = 0; i < 10; ++i) {
    absl::optional<size_t> value = queue.PopFront(0);
    EXPECT_EQ(queue.size(), 10lu);
    ASSERT_TRUE(value.has_value());
    EXPECT_EQ(value.value(), i);
  }
  for (size_t i = 0; i < 10; ++i) {
    absl::optional<size_t> value = queue.PopFront(1);
    EXPECT_EQ(queue.size(), 10lu);
    ASSERT_TRUE(value.has_value());
    EXPECT_EQ(value.value(), i);
  }
  for (size_t i = 0; i < 10; ++i) {
    absl::optional<size_t> value = queue.PopFront(2);
    EXPECT_EQ(queue.size(), 10 - i - 1);
    ASSERT_TRUE(value.has_value());
    EXPECT_EQ(value.value(), i);
  }
}

TEST(MultiHeadQueueTest, ThreeHeadsAddAllRemoveAll) {
  MultiHeadQueue<size_t> queue = MultiHeadQueue<size_t>(3);
  for (size_t i = 0; i < 10; ++i) {
    queue.PushBack(i);
    EXPECT_EQ(queue.size(), i + 1);
  }
  for (size_t i = 0; i < 10; ++i) {
    absl::optional<size_t> value1 = queue.PopFront(0);
    absl::optional<size_t> value2 = queue.PopFront(1);
    absl::optional<size_t> value3 = queue.PopFront(2);
    EXPECT_EQ(queue.size(), 10 - i - 1);
    ASSERT_TRUE(value1.has_value());
    ASSERT_TRUE(value2.has_value());
    ASSERT_TRUE(value3.has_value());
    EXPECT_EQ(value1.value(), i);
    EXPECT_EQ(value2.value(), i);
    EXPECT_EQ(value3.value(), i);
  }
}

TEST(MultiHeadQueueTest, HeadCopy) {
  MultiHeadQueue<size_t> queue = MultiHeadQueue<size_t>(1);
  for (size_t i = 0; i < 10; ++i) {
    queue.PushBack(i);
    EXPECT_EQ(queue.size(), i + 1);
  }
  queue.AddHead(0);
  EXPECT_EQ(queue.readers_count(), 2u);
  for (size_t i = 0; i < 10; ++i) {
    absl::optional<size_t> value1 = queue.PopFront(0);
    absl::optional<size_t> value2 = queue.PopFront(1);
    EXPECT_EQ(queue.size(), 10 - i - 1);
    ASSERT_TRUE(value1.has_value());
    ASSERT_TRUE(value2.has_value());
    EXPECT_EQ(value1.value(), i);
    EXPECT_EQ(value2.value(), i);
  }
}

}  // namespace
}  // namespace webrtc
