/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "rtc_tools/event_log_visualizer/joiner.h"

#include <algorithm>

#include "rtc_base/logging.h"
#include "rtc_tools/event_log_visualizer/analyzer.h"

namespace webrtc {

const int kNumMicrosecsPerSec = 1000000;

EventLogJoiner::EventLogJoiner(const ParsedRtcEventLogNew& log1,
                               const ParsedRtcEventLogNew& log2)
    : log1_(log1), log2_(log2) {
  for (const auto& config : log1.ice_candidate_pair_configs()) {
    log1_configs_[config.candidate_pair_id] =
        EventLogAnalyzer::GetCandidatePairLogDescriptionAsString(config);
  }
  for (const auto& config : log2.ice_candidate_pair_configs()) {
    log2_configs_[config.candidate_pair_id] =
        EventLogAnalyzer::GetCandidatePairLogDescriptionAsString(config);
  }

  std::cout << "# Log1 first timestamp: " << log1_.first_timestamp()
            << std::endl;
  std::cout << "# Log2 first timestamp: " << log2_.first_timestamp()
            << std::endl;
}

int64_t EventLogJoiner::ToCallTimeUs(int64_t timestamp,
                                     const ParsedRtcEventLogNew& log) {
  // TODO(zstein): first_timestamp is not the min timestamp in the new format.
  int64_t begin_time = log.first_timestamp();
  // std::cout << "# zz First timestamp: " << begin_time << " " << timestamp <<
  // " "
  //          << timestamp - begin_time << std::endl;

  return timestamp - begin_time;
}

float EventLogJoiner::ToCallTimeSec(int64_t timestamp,
                                    const ParsedRtcEventLogNew& log) {
  return static_cast<float>(ToCallTimeUs(timestamp, log)) / kNumMicrosecsPerSec;
}

void EventLogJoiner::CreateIceTransactionStateReached(
    PlotCollection* plot_collection) {
  struct IceTimestamp final {
    IceTimestamp(int64_t log_time_us, const ParsedRtcEventLogNew* log)
        : log_time_us_(log_time_us), log_(log) {
      RTC_DCHECK(log);
    }

    int64_t log_time_us() const { return log_time_us_; }

    int64_t CallTimeUs() const { return ToCallTimeUs(log_time_us_, *log_); }

    int64_t CallTimeS() const { return ToCallTimeSec(log_time_us_, *log_); }

    bool operator<(const IceTimestamp& other) {
      return log_time_us_ < other.log_time_us_ && log_ < other.log_;
    }

   private:
    int64_t log_time_us_;
    const ParsedRtcEventLogNew* log_;
  };

  struct IceTransaction final {
   public:
    absl::optional<uint32_t> log1_candidate_pair_id;
    absl::optional<uint32_t> log2_candidate_pair_id;

    using ConnectionId = std::pair<uint32_t, uint32_t>;
    ConnectionId connection_id() const {
      return std::make_pair(log1_candidate_pair_id.value_or(0),
                            log2_candidate_pair_id.value_or(0));
    }

    // TODO(zstein): Store in vector and provide named accessors
    std::vector<IceTimestamp> ping_sent;
    std::vector<IceTimestamp> ping_received;
    std::vector<IceTimestamp> response_sent;
    std::vector<IceTimestamp> response_received;

    int stage_reached() const {
      if (ping_sent.empty()) {
        return 0;
      } else if (ping_received.empty()) {
        return 1;
      } else if (response_sent.empty()) {
        return 2;
      } else if (response_received.empty()) {
        return 3;
      } else {
        return 4;
      }
    }

    void Update(const LoggedIceCandidatePairEvent& event,
                const ParsedRtcEventLogNew* log) {
      IceTimestamp timestamp(event.timestamp_us, log);

      switch (event.type) {
        case IceCandidatePairEventType::kCheckSent:
          ping_sent.emplace_back(event.timestamp_us, log);
          break;
        case IceCandidatePairEventType::kCheckReceived:
          ping_received.emplace_back(event.timestamp_us, log);
          break;
        case IceCandidatePairEventType::kCheckResponseSent:
          response_sent.emplace_back(event.timestamp_us, log);
          break;
        case IceCandidatePairEventType::kCheckResponseReceived:
          response_received.emplace_back(event.timestamp_us, log);
          break;
        case IceCandidatePairEventType::kNumValues:
          RTC_NOTREACHED();
          break;
      }
    }

    /*
    static std::string to_string(absl::optional x) {
      return x.has_value() ? to_string(*x) : "None";
    }

    std::string str() const {
      return "TransactionId(" +
             (log1_candidate_pair_id.has_value()
                  ? std::to_string(*log1_candidate_pair_id)
                  : "None") +
             ", " +
             (log2_candidate_pair_id.has_value()
                  ? std::to_string(*log2_candidate_pair_id)
                  : "None") +
             ", " +
             (ping_sent.has_value() ? std::to_string(*ping_sent) : "None") +
             ", " +
             (ping_received.has_value() ? std::to_string(*ping_received)
                                        : "None") +
             ", " +
             (response_sent.has_value() ? std::to_string(*response_sent)
                                        : "None") +
             ", " +
             (response_received.has_value() ? std::to_string(*response_received)
                                            : "None") +
             ")";
    }
    */
  };

  using TransactionId = uint32_t;

  std::unordered_map<TransactionId, IceTransaction> transactions;

  RTC_LOG(LS_INFO) << "# Building IceTransactions from log1 events";
  for (const auto& event : log1_.ice_candidate_pair_events()) {
    IceTransaction& transaction = transactions[event.transaction_id];
    transaction.log1_candidate_pair_id = event.candidate_pair_id;
    transaction.Update(event, &log1_);
  }

  RTC_LOG(LS_INFO) << "# Building IceTransactions from log2 events";
  for (const auto& event : log2_.ice_candidate_pair_events()) {
    IceTransaction& transaction = transactions[event.transaction_id];
    transaction.log2_candidate_pair_id = event.candidate_pair_id;
    transaction.Update(event, &log2_);
  }

  // transaction ids that are seen in both logs
  std::map<TransactionId, IceTransaction> interesting_transactions;
  for (const auto& transaction : transactions) {
    if (transaction.second.log1_candidate_pair_id &&
        transaction.second.log2_candidate_pair_id) {
      interesting_transactions.insert(transaction);
    }
  }

  /*
  for (const auto& kv : interesting_transactions) {
    std::cout << kv.first << ": " << kv.second.str() << std::endl;
  }
  //*/

  using IceTransactionVec = std::vector<IceTransaction>;
  std::map<IceTransaction::ConnectionId, IceTransactionVec> connections;
  for (const auto& transaction : interesting_transactions) {
    IceTransactionVec& vec = connections[transaction.second.connection_id()];
    vec.push_back(transaction.second);
  }

  /*
  for (const auto& connection : connections) {
    std::cout << "Connection(" << connection.first.first << ": "
              << connection.second.size() << ")" << std::endl;
  }
  //*/

  // Build TimeSeries and compute min/max x values.
  // x := time ping sent, y := stage reached
  for (const auto& connection : connections) {
    TimeSeries series(std::to_string(connection.first.first), LineStyle::kLine,
                      PointStyle::kHighlight);
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::min();
    for (const auto& transaction : connection.second) {
      if (!transaction.ping_sent.empty()) {
        float x =
            std::min_element(
                transaction.ping_sent.begin(), transaction.ping_sent.end(),
                [](const IceTimestamp& lhs, const IceTimestamp& rhs) {
                  return lhs.log_time_us() < rhs.log_time_us();
                })
                ->CallTimeS();
        x_min = std::min(x, x_min);
        x_max = std::max(x, x_max);
        float y = transaction.stage_reached();
        series.points.emplace_back(x, y);
      }
    }

    // TODO: Sort transactions by x value
    std::sort(series.points.begin(), series.points.end(),
              [](const TimeSeriesPoint& lhs, const TimeSeriesPoint& rhs) {
                return lhs.x < rhs.x;
              });

    Plot* plot = plot_collection->AppendNewPlot();
    plot->SetTitle("IceTransactionStateReached for connection pair " +
                   std::to_string(connection.first.first) + ", " +
                   std::to_string(connection.first.second));
    plot->AppendTimeSeries(std::move(series));
    plot->SetSuggestedYAxis(0, 4, "Stage Reached", 0, 1);
    plot->SetSuggestedXAxis(x_min, x_max, "Unnormalized Time (s)", 0.01, 0.01);
  }
}

void EventLogJoiner::CreateIceSequenceGraph(PlotCollection* plot_collection) {
  // const float typeMul = 0.01; // TODO
  // transaction_id -> TimeSeries

  // TODO(zstein): Maybe we want to have a mode that only plots things that go
  // wrong?

  std::map<uint32_t, TimeSeries> m;
  // TODO(zstein): candidate pair id is not stable across hosts
  std::map<uint32_t, uint32_t> transaction_id_to_candidate_pair_id;
  for (const auto& event : log1_.ice_candidate_pair_events()) {
    if (m.find(event.transaction_id) == m.end()) {
      m[event.transaction_id] =
          TimeSeries(std::to_string(event.candidate_pair_id) + " [" +
                         std::to_string(event.transaction_id) + "]",
                     LineStyle::kLine, PointStyle::kHighlight);
      transaction_id_to_candidate_pair_id[event.transaction_id] =
          event.candidate_pair_id;
    }
    float x = ToCallTimeSec(event.log_time_us(), log1_);
    float y = static_cast<float>(event.type);
    // float y = static_cast<float>(event.type) + 1.0f;
    m[event.transaction_id].points.emplace_back(x, y);
    std::cerr << "log1 (" << x << "," << y << ")" << std::endl;
  }

  for (const auto& event : log2_.ice_candidate_pair_events()) {
    if (m.find(event.transaction_id) == m.end()) {
      m[event.transaction_id] =
          TimeSeries(std::to_string(event.candidate_pair_id) + " [" +
                         std::to_string(event.transaction_id) + "]",
                     LineStyle::kLine, PointStyle::kHighlight);
    }
    float x = ToCallTimeSec(event.log_time_us(), log2_);
    float y = static_cast<float>(event.type) + 0.5f;
    // float y = -1 * (static_cast<float>(event.type) + 1.0f);
    m[event.transaction_id].points.emplace_back(x, y);
    std::cerr << "log2 (" << x << "," << y << ")" << std::endl;
  }

  float begin = std::numeric_limits<float>::max();
  float end = 0;
  int i = 0;
  std::map<uint32_t, Plot*> plots;
  for (auto& kv : m) {
    // auto first_x = kv.second.points.begin()->x;
    // for (auto& point : kv.second.points) {
    // point.x -= first_x; // shift time relative to first packet in series

    // Can't when checks actually happened with this.
    // point.x += 2 * i; // shift to avoid overlap

    // Can't tell when checks happened this way.
    // point.y += 5 * i; // shift up to avoid overlap
    //}
    ++i;

    /*
    // Skew makes this pretty useless.
    std::sort(kv.second.points.begin(), kv.second.points.end(),
              [](const TimeSeriesPoint& l, const TimeSeriesPoint& r) {
                return l.x < r.x;
              });
    */

    begin = std::min(
        begin,
        std::min_element(kv.second.points.begin(), kv.second.points.end(),
                         [](const TimeSeriesPoint& l,
                            const TimeSeriesPoint& r) { return l.x < r.x; })
            ->x);
    end = std::max(end, std::max_element(
                            kv.second.points.begin(), kv.second.points.end(),
                            [](const TimeSeriesPoint& l,
                               const TimeSeriesPoint& r) { return l.x < r.x; })
                            ->x);

    auto candidate_pair_id = transaction_id_to_candidate_pair_id[kv.first];
    Plot* plot;
    if (plots.find(candidate_pair_id) == plots.cend()) {
      plot = plot_collection->AppendNewPlot();
      plots[candidate_pair_id] = plot;

      plot->SetTitle("IceSequenceGraph for candidate_pair_id " +
                     std::to_string(candidate_pair_id));
      plot->SetXAxis(begin, end, "Time", 0.1f, 0.1f);
      plot->SetYAxis(0, 4 /*+ 5*i*/, "Event Type", 0.1f, 0.1f);

    } else {
      plot = plots[candidate_pair_id];
    }
    // TODO(zstein): Probably want a TimeSeries per candidate pair
    plot->AppendTimeSeries(std::move(kv.second));
  }
}

void EventLogJoiner::CreateIceTransactionPlots(
    PlotCollection* plot_collection) {
  using TransactionId = uint32_t;
  struct SourcedEvent {
    SourcedEvent(LoggedIceCandidatePairEvent event,
                 const ParsedRtcEventLogNew* log)
        : event(event), log(log) {}
    LoggedIceCandidatePairEvent event;
    const ParsedRtcEventLogNew* log;
  };
  using SourcedEventVec = std::vector<SourcedEvent>;
  std::unordered_map<TransactionId, SourcedEventVec> events_by_transaction_id;

  for (const auto& event : log1_.ice_candidate_pair_events()) {
    events_by_transaction_id[event.transaction_id].emplace_back(event, &log1_);
  }
  for (const auto& event : log2_.ice_candidate_pair_events()) {
    events_by_transaction_id[event.transaction_id].emplace_back(event, &log2_);
  }

  using CandidatePairId = uint32_t;
  // Multiple CandidatePairIds will point to the same plot if they share a
  // transaction.
  std::unordered_map<CandidatePairId, Plot*> plots;

  for (const auto& transaction_events : events_by_transaction_id) {
    const TransactionId& id = transaction_events.first;
    const SourcedEventVec& sourced_events = transaction_events.second;
    TimeSeries time_series(std::to_string(id), LineStyle::kLine,
                           PointStyle::kHighlight);
    std::set<CandidatePairId>
        candidate_pair_ids;  // TODO(zstein): Append to TimeSeries name?
    for (const auto& sourced_event : sourced_events) {
      candidate_pair_ids.insert(sourced_event.event.candidate_pair_id);
      float x =
          ToCallTimeSec(sourced_event.event.timestamp_us, *sourced_event.log);
      float y = static_cast<float>(sourced_event.event.type);
      time_series.points.emplace_back(x, y);
    }

    auto x_min_max = std::minmax_element(
        time_series.points.begin(), time_series.points.end(),
        [](const TimeSeriesPoint& lhs, const TimeSeriesPoint& rhs) {
          return lhs.x < rhs.x;
        });
    float x_min = x_min_max.first->x, x_max = x_min_max.second->x;

    Plot* plot;
    for (auto candidate_pair_id : candidate_pair_ids) {
      plot = plots[candidate_pair_id];
      if (plot) {
        break;
      }
    }
    if (plot == nullptr) {
      plot = plot_collection->AppendNewPlot();
      plot->SetTitle("IceTransactions for candidate_pair_id " +
                     std::to_string(*candidate_pair_ids.begin()) + ", ...");
    }
    for (auto candidate_pair_id : candidate_pair_ids) {
      plots[candidate_pair_id] = plot;
    }

    plot->AppendTimeSeries(std::move(time_series));
    plot->SetSuggestedYAxis(
        -1, static_cast<float>(IceCandidatePairEventType::kNumValues) + 1,
        "Numeric IceCandidatePairEvent Type", 0, 0);
    plot->SetSuggestedXAxis(x_min, x_max, "Unnormalized Time (s)", 0.01, 0.01);
  }
}

void EventLogJoiner::CreateActualIceSequenceDiagram(
    PlotCollection* plot_collection) {
  using TransactionId = uint32_t;
  struct SourcedEvent {
    SourcedEvent(LoggedIceCandidatePairEvent event,
                 const ParsedRtcEventLogNew* log)
        : event(event), log(log) {}
    LoggedIceCandidatePairEvent event;
    const ParsedRtcEventLogNew* log;
  };
  using SourcedEventVec = std::vector<SourcedEvent>;
  std::unordered_map<TransactionId, SourcedEventVec> events_by_transaction_id;

  for (const auto& event : log1_.ice_candidate_pair_events()) {
    events_by_transaction_id[event.transaction_id].emplace_back(event, &log1_);
  }
  for (const auto& event : log2_.ice_candidate_pair_events()) {
    events_by_transaction_id[event.transaction_id].emplace_back(event, &log2_);
  }

  using CandidatePairId = uint32_t;
  // Multiple CandidatePairIds will point to the same plot if they share a
  // transaction.
  std::unordered_map<CandidatePairId, Plot*> plots;

  for (const auto& transaction_events : events_by_transaction_id) {
    const TransactionId& id = transaction_events.first;
    const SourcedEventVec& sourced_events = transaction_events.second;
    TimeSeries time_series(std::to_string(id), LineStyle::kLine,
                           PointStyle::kHighlight);
    std::set<CandidatePairId>
        candidate_pair_ids;  // TODO(zstein): Append to TimeSeries name?
    for (const auto& sourced_event : sourced_events) {
      candidate_pair_ids.insert(sourced_event.event.candidate_pair_id);
      float x =
          ToCallTimeSec(sourced_event.event.timestamp_us, *sourced_event.log);
      float y = sourced_event.log == &log1_ ? 0 : 1;
      time_series.points.emplace_back(x, y);
    }

    auto x_min_max = std::minmax_element(
        time_series.points.begin(), time_series.points.end(),
        [](const TimeSeriesPoint& lhs, const TimeSeriesPoint& rhs) {
          return lhs.x < rhs.x;
        });
    float x_min = x_min_max.first->x, x_max = x_min_max.second->x;

    Plot* plot;
    for (auto candidate_pair_id : candidate_pair_ids) {
      plot = plots[candidate_pair_id];
      if (plot) {
        break;
      }
    }
    if (plot == nullptr) {
      plot = plot_collection->AppendNewPlot();
      plot->SetTitle("ActualIceSequenceDiagram for candidate_pair_ids " +
                     std::to_string(*candidate_pair_ids.begin()) + ", ...");
    }
    for (auto candidate_pair_id : candidate_pair_ids) {
      plots[candidate_pair_id] = plot;
    }

    plot->AppendTimeSeries(std::move(time_series));
    plot->SetSuggestedYAxis(0, 1, "Client", 0.1, 0.1);
    plot->SetSuggestedXAxis(x_min, x_max, "Unnormalized Time (s)", 0.01, 0.01);
  }
}

}  // namespace webrtc
