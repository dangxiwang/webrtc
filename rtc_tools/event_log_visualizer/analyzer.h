/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_TOOLS_EVENT_LOG_VISUALIZER_ANALYZER_H_
#define RTC_TOOLS_EVENT_LOG_VISUALIZER_ANALYZER_H_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "logging/rtc_event_log/rtc_event_log_parser.h"
#include "modules/audio_coding/audio_network_adaptor/include/audio_network_adaptor.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/rtcp_packet.h"
#include "rtc_base/function_view.h"
#include "rtc_tools/event_log_visualizer/plot_base.h"
#include "rtc_tools/event_log_visualizer/triage_notifications.h"

namespace webrtc {
namespace plotting {

struct LoggedRtpPacket {
  LoggedRtpPacket(uint64_t timestamp, RTPHeader header, size_t total_length)
      : timestamp(timestamp), header(header), total_length(total_length) {}
  uint64_t timestamp;
  // TODO(terelius): This allocates space for 15 CSRCs even if none are used.
  RTPHeader header;
  size_t total_length;
};

struct LoggedRtcpPacket {
  LoggedRtcpPacket(uint64_t timestamp,
                   RTCPPacketType rtcp_type,
                   std::unique_ptr<rtcp::RtcpPacket> rtcp_packet)
      : timestamp(timestamp), type(rtcp_type), packet(std::move(rtcp_packet)) {}
  uint64_t timestamp;
  RTCPPacketType type;
  std::unique_ptr<rtcp::RtcpPacket> packet;
};

struct LossBasedBweUpdate {
  uint64_t timestamp;
  int32_t new_bitrate;
  uint8_t fraction_loss;
  int32_t expected_packets;
};

struct AudioNetworkAdaptationEvent {
  uint64_t timestamp;
  AudioEncoderRuntimeConfig config;
};

class EventLogAnalyzer {
 public:
  // The EventLogAnalyzer keeps a reference to the ParsedRtcEventLog for the
  // duration of its lifetime. The ParsedRtcEventLog must not be destroyed or
  // modified while the EventLogAnalyzer is being used.
  explicit EventLogAnalyzer(const ParsedRtcEventLog& log);

  void CreatePacketGraph(PacketDirection desired_direction, Plot* plot);

  void CreateAccumulatedPacketsGraph(PacketDirection desired_direction,
                                     Plot* plot);

  void CreatePlayoutGraph(Plot* plot);

  void CreateAudioLevelGraph(Plot* plot);

  void CreateSequenceNumberGraph(Plot* plot);

  void CreateIncomingPacketLossGraph(Plot* plot);

  void CreateIncomingDelayDeltaGraph(Plot* plot);
  void CreateIncomingDelayGraph(Plot* plot);

  void CreateFractionLossGraph(Plot* plot);

  void CreateTotalBitrateGraph(PacketDirection desired_direction,
                               Plot* plot,
                               bool show_detector_state = false);

  void CreateStreamBitrateGraph(PacketDirection desired_direction, Plot* plot);

  void CreateSendSideBweSimulationGraph(Plot* plot);
  void CreateReceiveSideBweSimulationGraph(Plot* plot);

  void CreateNetworkDelayFeedbackGraph(Plot* plot);
  void CreatePacerDelayGraph(Plot* plot);
  void CreateTimestampGraph(Plot* plot);

  void CreateAudioEncoderTargetBitrateGraph(Plot* plot);
  void CreateAudioEncoderFrameLengthGraph(Plot* plot);
  void CreateAudioEncoderPacketLossGraph(Plot* plot);
  void CreateAudioEncoderEnableFecGraph(Plot* plot);
  void CreateAudioEncoderEnableDtxGraph(Plot* plot);
  void CreateAudioEncoderNumChannelsGraph(Plot* plot);
  void CreateAudioJitterBufferGraph(const std::string& replacement_file_name,
                                    int file_sample_rate_hz,
                                    Plot* plot);

  // Returns a vector of capture and arrival timestamps for the video frames
  // of the stream with the most number of frames.
  std::vector<std::pair<int64_t, int64_t>> GetFrameTimestamps() const;

  void CreateTriageNotifications();
  void PrintNotifications(FILE* file);

 private:
  class StreamId {
   public:
    StreamId(uint32_t ssrc, webrtc::PacketDirection direction)
        : ssrc_(ssrc), direction_(direction) {}
    bool operator<(const StreamId& other) const {
      return std::tie(ssrc_, direction_) <
             std::tie(other.ssrc_, other.direction_);
    }
    bool operator==(const StreamId& other) const {
      return std::tie(ssrc_, direction_) ==
             std::tie(other.ssrc_, other.direction_);
    }
    uint32_t GetSsrc() const { return ssrc_; }
    webrtc::PacketDirection GetDirection() const { return direction_; }

   private:
    uint32_t ssrc_;
    webrtc::PacketDirection direction_;
  };

  template <typename T>
  void CreateAccumulatedPacketsTimeSeries(
      PacketDirection desired_direction,
      Plot* plot,
      const std::map<StreamId, std::vector<T>>& packets,
      const std::string& label_prefix);

  bool IsRtxSsrc(StreamId stream_id) const;

  bool IsVideoSsrc(StreamId stream_id) const;

  bool IsAudioSsrc(StreamId stream_id) const;

  std::string GetStreamName(StreamId stream_id) const;

  rtc::Optional<uint32_t> EstimateRtpClockFrequency(
      const std::vector<LoggedRtpPacket>& packets) const;

  float ToCallTime(int64_t timestamp) const;

  void Notification(std::unique_ptr<TriageNotification> notification);

  const ParsedRtcEventLog& parsed_log_;

  // A list of SSRCs we are interested in analysing.
  // If left empty, all SSRCs will be considered relevant.
  std::vector<uint32_t> desired_ssrc_;

  // Tracks what each stream is configured for. Note that a single SSRC can be
  // in several sets. For example, the SSRC used for sending video over RTX
  // will appear in both video_ssrcs_ and rtx_ssrcs_. In the unlikely case that
  // an SSRC is reconfigured to a different media type mid-call, it will also
  // appear in multiple sets.
  std::set<StreamId> rtx_ssrcs_;
  std::set<StreamId> video_ssrcs_;
  std::set<StreamId> audio_ssrcs_;

  // Maps a stream identifier consisting of ssrc and direction to the parsed
  // RTP headers in that stream. Header extensions are parsed if the stream
  // has been configured.
  std::map<StreamId, std::vector<LoggedRtpPacket>> rtp_packets_;

  std::map<StreamId, std::vector<LoggedRtcpPacket>> rtcp_packets_;

  // Maps an SSRC to the timestamps of parsed audio playout events.
  std::map<uint32_t, std::vector<uint64_t>> audio_playout_events_;

  // Stores the timestamps for all log segments, in the form of associated start
  // and end events.
  std::vector<std::pair<uint64_t, uint64_t>> log_segments_;

  // A list of all updates from the send-side loss-based bandwidth estimator.
  std::vector<LossBasedBweUpdate> bwe_loss_updates_;

  std::vector<AudioNetworkAdaptationEvent> audio_network_adaptation_events_;

  std::vector<ParsedRtcEventLog::BweProbeClusterCreatedEvent>
      bwe_probe_cluster_created_events_;

  std::vector<ParsedRtcEventLog::BweProbeResultEvent> bwe_probe_result_events_;

  std::vector<ParsedRtcEventLog::BweDelayBasedUpdate> bwe_delay_updates_;

  std::vector<std::unique_ptr<TriageNotification>> notifications_;

  // Window and step size used for calculating moving averages, e.g. bitrate.
  // The generated data points will be |step_| microseconds apart.
  // Only events occuring at most |window_duration_| microseconds before the
  // current data point will be part of the average.
  uint64_t window_duration_;
  uint64_t step_;

  // First and last events of the log.
  uint64_t begin_time_;
  uint64_t end_time_;

  // Duration (in seconds) of log file.
  float call_duration_s_;
};

}  // namespace plotting
}  // namespace webrtc

#endif  // RTC_TOOLS_EVENT_LOG_VISUALIZER_ANALYZER_H_
