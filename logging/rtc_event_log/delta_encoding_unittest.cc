/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "logging/rtc_event_log/encoder/delta_encoding.h"

#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "rtc_base/arraysize.h"
#include "rtc_base/checks.h"
#include "rtc_base/random.h"
#include "test/gtest.h"

namespace webrtc {

void SetFixedLengthEncoderDeltaSignednessForTesting(bool signedness);

namespace {

enum class DeltaSignedness { kNoOverride, kForceUnsigned, kForceSigned };

void MaybeSetSignedness(DeltaSignedness signedness) {
  switch (signedness) {
    case DeltaSignedness::kNoOverride:
      return;
    case DeltaSignedness::kForceUnsigned:
      SetFixedLengthEncoderDeltaSignednessForTesting(false);
      return;
    case DeltaSignedness::kForceSigned:
      SetFixedLengthEncoderDeltaSignednessForTesting(true);
      return;
  }
  RTC_NOTREACHED();
}

uint64_t RandomWithMaxBitWidth(Random* prng, uint64_t max_width) {
  RTC_DCHECK_GE(max_width, 1u);
  RTC_DCHECK_LE(max_width, 64u);

  const uint64_t low = prng->Rand(std::numeric_limits<uint32_t>::max());
  const uint64_t high =
      max_width > 32u ? prng->Rand(std::numeric_limits<uint32_t>::max()) : 0u;

  const uint64_t random_before_mask = (high << 32) | low;

  if (max_width < 64) {
    return random_before_mask & ((static_cast<uint64_t>(1) << max_width) - 1);
  } else {
    return random_before_mask;
  }
}

// Encodes |values| based on |base|, then decodes the result and makes sure
// that it is equal to the original input.
// If |encoded_string| is non-null, the encoded result will also be written
// into it.
void TestEncodingAndDecoding(uint64_t base,
                             const std::vector<uint64_t>& values,
                             std::string* encoded_string = nullptr) {
  const std::string encoded = EncodeDeltas(base, values);
  if (encoded_string) {
    *encoded_string = encoded;
  }

  const std::vector<uint64_t> decoded =
      DecodeDeltas(encoded, base, values.size());

  EXPECT_EQ(decoded, values);
}

std::vector<uint64_t> CreateSequenceByFirstValue(uint64_t first,
                                                 size_t sequence_length) {
  std::vector<uint64_t> sequence(sequence_length);
  std::iota(sequence.begin(), sequence.end(), first);
  return sequence;
}

std::vector<uint64_t> CreateSequenceByLastValue(uint64_t last,
                                                size_t num_values) {
  const uint64_t first = last - num_values + 1;
  std::vector<uint64_t> result(num_values);
  std::iota(result.begin(), result.end(), first);
  return result;
}

// If |sequence_length| is greater than the number of deltas, the sequence of
// deltas will wrap around.
std::vector<uint64_t> CreateSequenceByDeltas(
    uint64_t first,
    const std::vector<uint64_t>& deltas,
    size_t sequence_length) {
  RTC_DCHECK_GE(sequence_length, 1);

  std::vector<uint64_t> sequence(sequence_length);

  uint64_t previous = first;
  for (size_t i = 0, next_delta_index = 0; i < sequence.size(); ++i) {
    sequence[i] = previous + deltas[next_delta_index];
    next_delta_index = (next_delta_index + 1) % deltas.size();
    previous = sequence[i];
  }

  return sequence;
}

// Tests of the delta encoding, parameterized by the number of values
// in the sequence created by the test.
class DeltaEncodingTest
    : public ::testing::TestWithParam<std::tuple<DeltaSignedness, size_t>> {
 public:
  DeltaEncodingTest()
      : signedness_(std::get<0>(GetParam())),
        num_of_values_(std::get<1>(GetParam())) {
    MaybeSetSignedness(signedness_);
  }
  ~DeltaEncodingTest() override = default;

  const DeltaSignedness signedness_;
  const uint64_t num_of_values_;
};

TEST_P(DeltaEncodingTest, AllValuesEqualToBaseValue) {
  const uint64_t base = 3432;
  std::vector<uint64_t> values(num_of_values_);
  std::fill(values.begin(), values.end(), base);
  std::string encoded;
  TestEncodingAndDecoding(base, values, &encoded);

  // Additional requirement - the encoding should be efficient in this
  // case - the empty string will be used.
  EXPECT_TRUE(encoded.empty());
}

TEST_P(DeltaEncodingTest, MinDeltaNoWrapAround) {
  const uint64_t base = 3432;

  const auto values = CreateSequenceByFirstValue(base + 1, num_of_values_);
  ASSERT_GT(values[values.size() - 1], base) << "Sanity; must not wrap around";

  TestEncodingAndDecoding(base, values);
}

TEST_P(DeltaEncodingTest, BigDeltaNoWrapAround) {
  const uint64_t kBigDelta = 132828;
  const uint64_t base = 3432;

  const auto values =
      CreateSequenceByFirstValue(base + kBigDelta, num_of_values_);
  ASSERT_GT(values[values.size() - 1], base) << "Sanity; must not wrap around";

  TestEncodingAndDecoding(base, values);
}

TEST_P(DeltaEncodingTest, MaxDeltaNoWrapAround) {
  const uint64_t base = 3432;

  const auto values = CreateSequenceByLastValue(
      std::numeric_limits<uint64_t>::max(), num_of_values_);
  ASSERT_GT(values[values.size() - 1], base) << "Sanity; must not wrap around";

  TestEncodingAndDecoding(base, values);
}

TEST_P(DeltaEncodingTest, MinDeltaWithWrapAround) {
  const uint64_t base = std::numeric_limits<uint64_t>::max();

  const auto values = CreateSequenceByDeltas(0, {10, 3}, num_of_values_);
  ASSERT_LT(values[values.size() - 1], base) << "Sanity; must wrap around";

  TestEncodingAndDecoding(base, values);
}

// Suppress "integral constant overflow" warning; this is the test's focus.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4307)
#endif
TEST_P(DeltaEncodingTest, BigDeltaWithWrapAround) {
  const uint64_t kBigDelta = 132828;
  const uint64_t base = std::numeric_limits<uint64_t>::max() - kBigDelta + 3;

  const auto values =
      CreateSequenceByFirstValue(base + kBigDelta, num_of_values_);
  ASSERT_LT(values[values.size() - 1], base) << "Sanity; must wrap around";

  TestEncodingAndDecoding(base, values);
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif

TEST_P(DeltaEncodingTest, MaxDeltaWithWrapAround) {
  const uint64_t base = 3432;

  const auto values = CreateSequenceByLastValue(3, num_of_values_);
  ASSERT_LT(values[values.size() - 1], base) << "Sanity; must wrap around";

  TestEncodingAndDecoding(base, values);
}

// If num_of_values_ == 1, a zero delta will yield an empty string; that's
// already covered by AllValuesEqualToBaseValue, but it doesn't hurt to test
// again. For all other cases, we have a new test.
TEST_P(DeltaEncodingTest, ZeroDelta) {
  const uint64_t base = 3432;

  // Arbitrary sequence of deltas with intentional zero deltas, as well as
  // consecutive zeros.
  const std::vector<uint64_t> deltas = {0,      312, 11, 1,  1, 0, 0, 12,
                                        400321, 3,   3,  12, 5, 0, 6};
  const auto values = CreateSequenceByDeltas(base, deltas, num_of_values_);

  TestEncodingAndDecoding(base, values);
}

INSTANTIATE_TEST_CASE_P(
    SignednessOverrideAndNumberOfValuesInSequence,
    DeltaEncodingTest,
    ::testing::Combine(::testing::Values(DeltaSignedness::kNoOverride,
                                         DeltaSignedness::kForceUnsigned,
                                         DeltaSignedness::kForceSigned),
                       ::testing::Values(1, 2, 100, 10000)));

// Tests over the quality of the compression (as opposed to its correctness).
// Not to be confused with tests of runtime efficiency.
class DeltaEncodingCompressionQualityTest
    : public ::testing::TestWithParam<
          std::tuple<DeltaSignedness, uint64_t, uint64_t>> {
 public:
  DeltaEncodingCompressionQualityTest()
      : signedness_(std::get<0>(GetParam())),
        delta_max_bit_width_(std::get<1>(GetParam())),
        num_of_values_(std::get<2>(GetParam())) {
    MaybeSetSignedness(signedness_);
  }

  ~DeltaEncodingCompressionQualityTest() override = default;

  const DeltaSignedness signedness_;
  const uint64_t delta_max_bit_width_;
  const uint64_t num_of_values_;
};

// If no wrap-around occurs in the stream, the width of the values does not
// matter to compression performance; only the deltas matter.
TEST_P(DeltaEncodingCompressionQualityTest,
       BaseDoesNotAffectEfficiencyIfNoWrapAround) {
  Random prng(3012);
  std::vector<uint64_t> deltas(num_of_values_);
  for (size_t i = 0; i < deltas.size(); ++i) {
    deltas[i] = RandomWithMaxBitWidth(&prng, delta_max_bit_width_);
  }

  // 1. Bases which will not produce a wrap-around.
  // 2. The last base - 0xffffffffffffffff - does cause a wrap-around, but
  //    that still works, because the width is 64 anyway, and does not
  //    need to be conveyed explicitly in the encoding header.
  const uint64_t bases[] = {0, 0x55, 0xffffffff,
                            std::numeric_limits<uint64_t>::max()};

  std::string encodings[arraysize(bases)];

  for (size_t i = 0; i < arraysize(bases); ++i) {
    const auto values =
        CreateSequenceByDeltas(bases[i], deltas, num_of_values_);
    // Produce the encoding and write it to encodings[i].
    // By using TestEncodingAndDecoding() to do this, we also sanity-test
    // the encoding/decoding, though that is not the test's focus.
    TestEncodingAndDecoding(bases[i], values, &encodings[i]);
  }

  // Test focus - all of the encodings should be the same, as they are based
  // on the same delta sequence, and do not contain a wrap-around.
  for (size_t i = 1; i < arraysize(encodings); ++i) {
    EXPECT_EQ(encodings[i], encodings[0]);
  }
}

INSTANTIATE_TEST_CASE_P(
    SignednessOverrideAndDeltaMaxBitWidthAndNumberOfValuesInSequence,
    DeltaEncodingCompressionQualityTest,
    ::testing::Combine(
        ::testing::Values(DeltaSignedness::kNoOverride,
                          DeltaSignedness::kForceUnsigned,
                          DeltaSignedness::kForceSigned),
        ::testing::Values(1, 4, 8, 15, 16, 17, 31, 32, 33, 63, 64),
        ::testing::Values(1, 2, 100, 10000)));

// Similar to DeltaEncodingTest, but instead of semi-surgically producing
// specific cases, produce large amount of semi-realistic inputs.
class DeltaEncodingFuzzerLikeTest
    : public ::testing::TestWithParam<
          std::tuple<DeltaSignedness, uint64_t, uint64_t>> {
 public:
  DeltaEncodingFuzzerLikeTest()
      : signedness_(std::get<0>(GetParam())),
        delta_max_bit_width_(std::get<1>(GetParam())),
        num_of_values_(std::get<2>(GetParam())) {
    MaybeSetSignedness(signedness_);
  }

  ~DeltaEncodingFuzzerLikeTest() override = default;

  const DeltaSignedness signedness_;
  const uint64_t delta_max_bit_width_;
  const uint64_t num_of_values_;
};

TEST_P(DeltaEncodingFuzzerLikeTest, Test) {
  const uint64_t base = 3432;

  Random prng(1983);
  std::vector<uint64_t> deltas(num_of_values_);
  for (size_t i = 0; i < deltas.size(); ++i) {
    deltas[i] = RandomWithMaxBitWidth(&prng, delta_max_bit_width_);
  }

  const auto values = CreateSequenceByDeltas(base, deltas, num_of_values_);

  TestEncodingAndDecoding(base, values);
}

INSTANTIATE_TEST_CASE_P(
    SignednessOverrideAndDeltaMaxBitWidthAndNumberOfValuesInSequence,
    DeltaEncodingFuzzerLikeTest,
    ::testing::Combine(
        ::testing::Values(DeltaSignedness::kNoOverride,
                          DeltaSignedness::kForceUnsigned,
                          DeltaSignedness::kForceSigned),
        ::testing::Values(1, 4, 8, 15, 16, 17, 31, 32, 33, 63, 64),
        ::testing::Values(1, 2, 100, 10000)));

class DeltaEncodingSpecificEdgeCasesTest : public ::testing::Test {
 public:
  ~DeltaEncodingSpecificEdgeCasesTest() override = default;
};

// This case is special because it produces identical forward/backward deltas.
TEST_F(DeltaEncodingSpecificEdgeCasesTest, SignedDeltaWithOnlyTopBitOn) {
  MaybeSetSignedness(DeltaSignedness::kForceSigned);

  const uint64_t base = 3432;

  const uint64_t delta = static_cast<uint64_t>(1) << 63;
  const std::vector<uint64_t> values = {base + delta};

  TestEncodingAndDecoding(base, values);
}

TEST_F(DeltaEncodingSpecificEdgeCasesTest, MaximumUnsignedDelta) {
  MaybeSetSignedness(DeltaSignedness::kForceUnsigned);

  const uint64_t base = (static_cast<uint64_t>(1) << 63) + 0x123;

  const std::vector<uint64_t> values = {base - 1};

  TestEncodingAndDecoding(base, values);
}

}  // namespace
}  // namespace webrtc
