/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_coding/neteq/tools/neteq_test_factory.h"

#include <errno.h>
#include <limits.h>  // For ULONG_MAX returned by strtoul.
#include <stdio.h>
#include <stdlib.h>  // For strtoul.
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "absl/memory/memory.h"
#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "modules/audio_coding/neteq/include/neteq.h"
#include "modules/audio_coding/neteq/tools/fake_decode_from_file.h"
#include "modules/audio_coding/neteq/tools/input_audio_file.h"
#include "modules/audio_coding/neteq/tools/neteq_delay_analyzer.h"
#include "modules/audio_coding/neteq/tools/neteq_event_log_input.h"
#include "modules/audio_coding/neteq/tools/neteq_packet_source_input.h"
#include "modules/audio_coding/neteq/tools/neteq_replacement_input.h"
#include "modules/audio_coding/neteq/tools/neteq_stats_getter.h"
#include "modules/audio_coding/neteq/tools/neteq_stats_plotter.h"
#include "modules/audio_coding/neteq/tools/neteq_test.h"
#include "modules/audio_coding/neteq/tools/output_audio_file.h"
#include "modules/audio_coding/neteq/tools/output_wav_file.h"
#include "modules/audio_coding/neteq/tools/rtp_file_source.h"
#include "rtc_base/checks.h"
#include "rtc_base/flags.h"
#include "rtc_base/ref_counted_object.h"
#include "test/function_audio_decoder_factory.h"
#include "test/testsupport/file_utils.h"

namespace webrtc {
namespace test {
namespace {

absl::optional<int> CodecSampleRate(
    uint8_t payload_type,
    webrtc::test::NetEqTestFactory::Config config) {
  if (payload_type == config.pcmu || payload_type == config.pcma ||
      payload_type == config.ilbc || payload_type == config.pcm16b ||
      payload_type == config.cn_nb || payload_type == config.avt)
    return 8000;
  if (payload_type == config.isac || payload_type == config.pcm16b_wb ||
      payload_type == config.g722 || payload_type == config.cn_wb ||
      payload_type == config.avt_16)
    return 16000;
  if (payload_type == config.isac_swb || payload_type == config.pcm16b_swb32 ||
      payload_type == config.cn_swb32 || payload_type == config.avt_32)
    return 32000;
  if (payload_type == config.opus || payload_type == config.pcm16b_swb48 ||
      payload_type == config.cn_swb48 || payload_type == config.avt_48)
    return 48000;
  if (payload_type == config.red)
    return 0;
  return absl::nullopt;
}

}  // namespace

// A callback class which prints whenver the inserted packet stream changes
// the SSRC.
class SsrcSwitchDetector : public NetEqPostInsertPacket {
 public:
  // Takes a pointer to another callback object, which will be invoked after
  // this object finishes. This does not transfer ownership, and null is a
  // valid value.
  explicit SsrcSwitchDetector(NetEqPostInsertPacket* other_callback)
      : other_callback_(other_callback) {}

  void AfterInsertPacket(const NetEqInput::PacketData& packet,
                         NetEq* neteq) override {
    if (last_ssrc_ && packet.header.ssrc != *last_ssrc_) {
      std::cout << "Changing streams from 0x" << std::hex << *last_ssrc_
                << " to 0x" << std::hex << packet.header.ssrc << std::dec
                << " (payload type "
                << static_cast<int>(packet.header.payloadType) << ")"
                << std::endl;
    }
    last_ssrc_ = packet.header.ssrc;
    if (other_callback_) {
      other_callback_->AfterInsertPacket(packet, neteq);
    }
  }

 private:
  NetEqPostInsertPacket* other_callback_;
  absl::optional<uint32_t> last_ssrc_;
};

NetEqTestFactory::NetEqTestFactory() = default;
NetEqTestFactory::~NetEqTestFactory() = default;

NetEqTestFactory::Config::Config() = default;
NetEqTestFactory::Config::Config(const Config& other) = default;
NetEqTestFactory::Config::~Config() = default;

std::unique_ptr<NetEqTest> NetEqTestFactory::InitializeTest(
    std::string input_file_name,
    std::string output_file_name,
    const Config& config) {
  // Gather RTP header extensions in a map.
  NetEqPacketSourceInput::RtpHeaderExtensionMap rtp_ext_map = {
      {config.audio_level, kRtpExtensionAudioLevel},
      {config.abs_send_time, kRtpExtensionAbsoluteSendTime},
      {config.transport_seq_no, kRtpExtensionTransportSequenceNumber},
      {config.video_content_type, kRtpExtensionVideoContentType},
      {config.video_timing, kRtpExtensionVideoTiming}};

  std::unique_ptr<NetEqInput> input;
  if (RtpFileSource::ValidRtpDump(input_file_name) ||
      RtpFileSource::ValidPcap(input_file_name)) {
    input.reset(new NetEqRtpDumpInput(input_file_name, rtp_ext_map,
                                      config.ssrc_filter));
  } else {
    input.reset(new NetEqEventLogInput(input_file_name, config.ssrc_filter));
  }

  std::cout << "Input file: " << input_file_name << std::endl;
  RTC_CHECK(input) << "Cannot open input file";
  RTC_CHECK(!input->ended()) << "Input file is empty";

  // Check the sample rate.
  absl::optional<int> sample_rate_hz;
  std::set<std::pair<int, uint32_t>> discarded_pt_and_ssrc;
  while (absl::optional<RTPHeader> first_rtp_header = input->NextHeader()) {
    RTC_DCHECK(first_rtp_header);
    sample_rate_hz = CodecSampleRate(first_rtp_header->payloadType, config);
    if (sample_rate_hz) {
      std::cout << "Found valid packet with payload type "
                << static_cast<int>(first_rtp_header->payloadType)
                << " and SSRC 0x" << std::hex << first_rtp_header->ssrc
                << std::dec << std::endl;
      break;
    }
    // Discard this packet and move to the next. Keep track of discarded payload
    // types and SSRCs.
    discarded_pt_and_ssrc.emplace(first_rtp_header->payloadType,
                                  first_rtp_header->ssrc);
    input->PopPacket();
  }
  if (!discarded_pt_and_ssrc.empty()) {
    std::cout << "Discarded initial packets with the following payload types "
                 "and SSRCs:"
              << std::endl;
    for (const auto& d : discarded_pt_and_ssrc) {
      std::cout << "PT " << d.first << "; SSRC 0x" << std::hex
                << static_cast<int>(d.second) << std::dec << std::endl;
    }
  }
  if (!sample_rate_hz) {
    std::cout << "Cannot find any packets with known payload types"
              << std::endl;
    RTC_NOTREACHED();
  }

  // Open the output file now that we know the sample rate. (Rate is only needed
  // for wav files.)
  std::unique_ptr<AudioSink> output;
  if (output_file_name.size() >= 4 &&
      output_file_name.substr(output_file_name.size() - 4) == ".wav") {
    // Open a wav file.
    output.reset(new OutputWavFile(output_file_name, *sample_rate_hz));
  } else {
    // Open a pcm file.
    output.reset(new OutputAudioFile(output_file_name));
  }

  std::cout << "Output file: " << output_file_name << std::endl;

  NetEqTest::DecoderMap codecs = NetEqTest::StandardDecoderMap();

  rtc::scoped_refptr<AudioDecoderFactory> decoder_factory =
      CreateBuiltinAudioDecoderFactory();

  // Check if a replacement audio file was provided.
  if (config.replacement_audio_file.size() > 0) {
    // Find largest unused payload type.
    int replacement_pt = 127;
    while (codecs.find(replacement_pt) != codecs.end()) {
      --replacement_pt;
      RTC_CHECK_GE(replacement_pt, 0);
    }

    auto std_set_int32_to_uint8 = [](const std::set<int32_t>& a) {
      std::set<uint8_t> b;
      for (auto& x : a) {
        b.insert(static_cast<uint8_t>(x));
      }
      return b;
    };

    std::set<uint8_t> cn_types = std_set_int32_to_uint8(
        {config.cn_nb, config.cn_wb, config.cn_swb32, config.cn_swb48});
    std::set<uint8_t> forbidden_types =
        std_set_int32_to_uint8({config.g722, config.red, config.avt,
                                config.avt_16, config.avt_32, config.avt_48});
    input.reset(new NetEqReplacementInput(std::move(input), replacement_pt,
                                          cn_types, forbidden_types));

    // Note that capture-by-copy implies that the lambda captures the value of
    // decoder_factory before it's reassigned on the left-hand side.
    decoder_factory = new rtc::RefCountedObject<FunctionAudioDecoderFactory>(
        [decoder_factory, config](
            const SdpAudioFormat& format,
            absl::optional<AudioCodecPairId> codec_pair_id) {
          std::unique_ptr<AudioDecoder> decoder =
              decoder_factory->MakeAudioDecoder(format, codec_pair_id);
          if (!decoder && format.name == "replacement") {
            decoder = absl::make_unique<FakeDecodeFromFile>(
                absl::make_unique<InputAudioFile>(
                    config.replacement_audio_file),
                format.clockrate_hz, format.num_channels > 1);
          }
          return decoder;
        });

    RTC_CHECK(
        codecs.insert({replacement_pt, SdpAudioFormat("replacement", 48000, 1)})
            .second);
  }

  // Create a text log file if needed.
  std::unique_ptr<std::ofstream> text_log;
  if (config.textlog) {
    text_log =
        absl::make_unique<std::ofstream>(output_file_name + ".text_log.txt");
  }

  NetEqTest::Callbacks callbacks;
  stats_plotter_.reset(
      new NetEqStatsPlotter(config.matlabplot, config.pythonplot,
                            config.concealment_events, output_file_name));

  ssrc_switch_detector_.reset(
      new SsrcSwitchDetector(stats_plotter_->stats_getter()->delay_analyzer()));
  callbacks.post_insert_packet = ssrc_switch_detector_.get();
  callbacks.get_audio_callback = stats_plotter_->stats_getter();
  callbacks.simulation_ended_callback = stats_plotter_.get();
  NetEq::Config neteq_config;
  neteq_config.sample_rate_hz = *sample_rate_hz;
  neteq_config.max_packets_in_buffer = config.max_nr_packets_in_buffer;
  neteq_config.enable_fast_accelerate = config.enable_fast_accelerate;
  return absl::make_unique<NetEqTest>(neteq_config, decoder_factory, codecs,
                                      std::move(text_log), std::move(input),
                                      std::move(output), callbacks);
}

}  // namespace test
}  // namespace webrtc
