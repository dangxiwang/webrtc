/*
 *  Copyright (c) 2021 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#include "net/dcsctp/packet/chunk_validators.h"

#include <utility>

#include "rtc_base/gunit.h"
#include "test/gmock.h"

namespace dcsctp {
namespace {
using ::testing::ElementsAre;

TEST(ChunkValidatorsTest, RemovesInvalidGapAckBlockFromSack) {
  SackChunk sack(TSN(123), /*a_rwnd=*/456,
                 {SackChunk::GapAckBlock(2, 3), SackChunk::GapAckBlock(6, 4)},
                 {});

  SackChunk clean = ChunkValidators::Clean(std::move(sack));

  EXPECT_THAT(clean.gap_ack_blocks(),
              ElementsAre(SackChunk::GapAckBlock(2, 3)));
}

TEST(ChunkValidatorsTest, SortsGapAckBlocksInOrder) {
  SackChunk sack(TSN(123), /*a_rwnd=*/456,
                 {SackChunk::GapAckBlock(6, 7), SackChunk::GapAckBlock(3, 4)},
                 {});

  SackChunk clean = ChunkValidators::Clean(std::move(sack));

  EXPECT_THAT(
      clean.gap_ack_blocks(),
      ElementsAre(SackChunk::GapAckBlock(3, 4), SackChunk::GapAckBlock(6, 7)));
}

TEST(ChunkValidatorsTest, MergesAdjacentBlocks) {
  SackChunk sack(TSN(123), /*a_rwnd=*/456,
                 {SackChunk::GapAckBlock(3, 4), SackChunk::GapAckBlock(4, 5)},
                 {});

  SackChunk clean = ChunkValidators::Clean(std::move(sack));

  EXPECT_THAT(clean.gap_ack_blocks(),
              ElementsAre(SackChunk::GapAckBlock(3, 5)));
}

TEST(ChunkValidatorsTest, MergesCompletelyOverlapping) {
  SackChunk sack(TSN(123), /*a_rwnd=*/456,
                 {SackChunk::GapAckBlock(3, 10), SackChunk::GapAckBlock(4, 5)},
                 {});

  SackChunk clean = ChunkValidators::Clean(std::move(sack));

  EXPECT_THAT(clean.gap_ack_blocks(),
              ElementsAre(SackChunk::GapAckBlock(3, 10)));
}

TEST(ChunkValidatorsTest, MergesBlocksStartingWithSameStartOffset) {
  SackChunk sack(TSN(123), /*a_rwnd=*/456,
                 {SackChunk::GapAckBlock(3, 7), SackChunk::GapAckBlock(3, 5),
                  SackChunk::GapAckBlock(3, 9)},
                 {});

  SackChunk clean = ChunkValidators::Clean(std::move(sack));

  EXPECT_THAT(clean.gap_ack_blocks(),
              ElementsAre(SackChunk::GapAckBlock(3, 9)));
}

TEST(ChunkValidatorsTest, MergesBlocksPartiallyOverlapping) {
  SackChunk sack(TSN(123), /*a_rwnd=*/456,
                 {SackChunk::GapAckBlock(3, 7), SackChunk::GapAckBlock(5, 9)},
                 {});

  SackChunk clean = ChunkValidators::Clean(std::move(sack));

  EXPECT_THAT(clean.gap_ack_blocks(),
              ElementsAre(SackChunk::GapAckBlock(3, 9)));
}

}  // namespace
}  // namespace dcsctp
