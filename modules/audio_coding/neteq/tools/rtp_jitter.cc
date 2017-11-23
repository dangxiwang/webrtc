/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "modules/rtp_rtcp/source/byte_io.h"
#include "rtc_base/buffer.h"
#include "rtc_base/flags.h"
#include "typedefs.h"  // NOLINT(build/include)

namespace webrtc {
namespace test {
namespace {

DEFINE_bool(help, false, "Print help message");

constexpr size_t kRtpDumpHeaderLength = 8;

std::unique_ptr<rtc::Buffer> ReadNextPacket(FILE* file) {
  uint8_t header[kRtpDumpHeaderLength];
  if (fread(header, sizeof(kRtpDumpHeaderLength), 1, file) != 1) {
    return std::unique_ptr<rtc::Buffer>();
  }
  // Get length field. This is the total length for this packet written to file,
  // including the kRtpDumpHeaderLength bytes already read.
  const uint16_t len = ByteReader<uint16_t>::ReadBigEndian(header);
  std::unique_ptr<rtc::Buffer> buffer(new rtc::Buffer(len));
  // Copy header to first part of buffer.
  memcpy(buffer->data(), header, kRtpDumpHeaderLength);
  // Read remaining data from file directly into buffer.
  if (fread(buffer->data() + kRtpDumpHeaderLength, len - kRtpDumpHeaderLength,
            1, file) != 1) {
    return std::unique_ptr<rtc::Buffer>();
  }
  return buffer;
}

using PacketAndTime = std::pair<std::unique_ptr<rtc::Buffer>, int>;

void WritePacket(const PacketAndTime& packet, FILE* file) {
  // Write the first 4 bytes from the original packet.
  const auto* payload_ptr = packet.first->data();
  RTC_CHECK_EQ(fwrite(payload_ptr, 4, 1, file), 1);
  payload_ptr += 4;

  // Convert the new time offset to network endian, and write to file.
  uint8_t time[sizeof(uint32_t)];
  ByteWriter<uint32_t, sizeof(uint32_t)>::WriteBigEndian(time, packet.second);
  RTC_CHECK_EQ(fwrite(time, sizeof(uint32_t), 1, file), 1);
  payload_ptr += 4;  // Skip the old time in the original payload.

  // Write the remaining part of the payload.
  RTC_DCHECK_EQ(payload_ptr - packet.first->data(), kRtpDumpHeaderLength);
  RTC_CHECK_EQ(
      fwrite(payload_ptr, packet.first->size() - kRtpDumpHeaderLength, 1, file),
      1);
}

int RunRtpJitter(int argc, char* argv[]) {
  const std::string program_name = argv[0];
  const std::string usage =
      "Tool for alternating the arrival times in an RTP dump file.\n"
      "Example usage:\n" +
      program_name + " input.rtp arrival_times_ms.txt output.rtp\n\n";
  if (rtc::FlagList::SetFlagsFromCommandLine(&argc, argv, true) || FLAG_help ||
      argc != 4) {
    printf("%s", usage.c_str());
    return FLAG_help ? 0 : 1;
  }

  printf("Input RTP file: %s\n", argv[1]);
  FILE* in_file = fopen(argv[1], "rb");
  RTC_CHECK(in_file) << "Could not open file " << argv[1] << " for reading";
  printf("Timing file: %s\n", argv[2]);
  std::ifstream timing_file(argv[2]);
  printf("Output file: %s\n", argv[3]);
  FILE* out_file = fopen(argv[3], "wb");
  RTC_CHECK(out_file) << "Could not open file " << argv[2] << " for writing";

  // Copy the RTP file header to the output file.
  char header_string[30];
  RTC_CHECK(fgets(header_string, 30, in_file));
  fprintf(out_file, "%s", header_string);
  uint8_t file_header[16];
  RTC_CHECK_EQ(fread(file_header, sizeof(file_header), 1, in_file), 1);
  RTC_CHECK_EQ(fwrite(file_header, sizeof(file_header), 1, out_file), 1);

  // Read all time values from the timing file. Store in a vector.
  std::vector<int> new_arrival_times;
  int new_time;
  while (timing_file >> new_time) {
    new_arrival_times.push_back(new_time);
  }

  // Read all packets from the input RTP file, but no more than the number of
  // new time values. Store RTP packets together with new time values.
  auto time_it = new_arrival_times.begin();
  std::vector<PacketAndTime> packets;
  while (auto packet = ReadNextPacket(in_file)) {
    if (time_it == new_arrival_times.end()) {
      break;
    }
    RTC_DCHECK(packet);
    packets.emplace_back(std::move(packet), *time_it);
    ++time_it;
  }

  // Sort on new time values.
  std::sort(packets.begin(), packets.end(),
            [](const PacketAndTime& a, const PacketAndTime& b) {
              return a.second < b.second;
            });

  // Write packets to output file.
  for (auto& p : packets) {
    WritePacket(p, out_file);
  }

  fclose(in_file);
  fclose(out_file);
  return 0;
}

}  // namespace
}  // namespace test
}  // namespace webrtc

int main(int argc, char* argv[]) {
  return webrtc::test::RunRtpJitter(argc, argv);
}
