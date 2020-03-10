/*
 *  Copyright 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// This file contains structures used for retrieving statistics from an ongoing
// libjingle session.

#ifndef API_STATS_TYPES_H_
#define API_STATS_TYPES_H_

#include <algorithm>
#include <list>
#include <map>
#include <string>
#include <vector>

#include "api/scoped_refptr.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/ref_count.h"
#include "rtc_base/string_encode.h"
#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/system/rtc_export.h"

namespace webrtc {

class RTC_EXPORT StatsReport {
 public:
  // Indicates whether a track is for sending or receiving.
  // Used in reports for audio/video tracks.
  enum Direction {
    kSend = 0,
    kReceive,
  };

  enum StatsType {
    // StatsReport types.
    // A StatsReport of |type| = "googSession" contains overall information
    // about the thing libjingle calls a session (which may contain one
    // or more RTP sessions.
    kStatsReportTypeSession,

    // A StatsReport of |type| = "googTransport" contains information
    // about a libjingle "transport".
    kStatsReportTypeTransport,

    // A StatsReport of |type| = "googComponent" contains information
    // about a libjingle "channel" (typically, RTP or RTCP for a transport).
    // This is intended to be the same thing as an ICE "Component".
    kStatsReportTypeComponent,

    // A StatsReport of |type| = "googCandidatePair" contains information
    // about a libjingle "connection" - a single source/destination port pair.
    // This is intended to be the same thing as an ICE "candidate pair".
    kStatsReportTypeCandidatePair,

    // A StatsReport of |type| = "VideoBWE" is statistics for video Bandwidth
    // Estimation, which is global per-session.  The |id| field is "bweforvideo"
    // (will probably change in the future).
    kStatsReportTypeBwe,

    // A StatsReport of |type| = "ssrc" is statistics for a specific rtp stream.
    // The |id| field is the SSRC in decimal form of the rtp stream.
    kStatsReportTypeSsrc,

    // A StatsReport of |type| = "remoteSsrc" is statistics for a specific
    // rtp stream, generated by the remote end of the connection.
    kStatsReportTypeRemoteSsrc,

    // A StatsReport of |type| = "googTrack" is statistics for a specific media
    // track. The |id| field is the track id.
    kStatsReportTypeTrack,

    // A StatsReport of |type| = "localcandidate" or "remotecandidate" is
    // attributes on a specific ICE Candidate. It links to its connection pair
    // by candidate id. The string value is taken from
    // http://w3c.github.io/webrtc-stats/#rtcstatstype-enum*.
    kStatsReportTypeIceLocalCandidate,
    kStatsReportTypeIceRemoteCandidate,

    // A StatsReport of |type| = "googCertificate" contains an SSL certificate
    // transmitted by one of the endpoints of this connection.  The |id| is
    // controlled by the fingerprint, and is used to identify the certificate in
    // the Channel stats (as "googLocalCertificateId" or
    // "googRemoteCertificateId") and in any child certificates (as
    // "googIssuerId").
    kStatsReportTypeCertificate,

    // A StatsReport of |type| = "datachannel" with statistics for a
    // particular DataChannel.
    kStatsReportTypeDataChannel,
  };

  enum StatsValueName {
    kStatsValueNameActiveConnection,
    kStatsValueNameAecDivergentFilterFraction,
    kStatsValueNameAudioInputLevel,
    kStatsValueNameAudioOutputLevel,
    kStatsValueNameBytesReceived,
    kStatsValueNameBytesSent,
    kStatsValueNameCodecImplementationName,
    kStatsValueNameConcealedSamples,
    kStatsValueNameConcealmentEvents,
    kStatsValueNameDataChannelId,
    kStatsValueNameFramesDecoded,
    kStatsValueNameFramesEncoded,
    kStatsValueNameJitterBufferDelay,
    kStatsValueNameMediaType,
    kStatsValueNamePacketsLost,
    kStatsValueNamePacketsReceived,
    kStatsValueNamePacketsSent,
    kStatsValueNameProtocol,
    kStatsValueNameQpSum,
    kStatsValueNameReceiving,
    kStatsValueNameSelectedCandidatePairId,
    kStatsValueNameSsrc,
    kStatsValueNameState,
    kStatsValueNameTotalAudioEnergy,
    kStatsValueNameTotalSamplesDuration,
    kStatsValueNameTotalSamplesReceived,
    kStatsValueNameTransportId,
    kStatsValueNameSentPingRequestsTotal,
    kStatsValueNameSentPingRequestsBeforeFirstResponse,
    kStatsValueNameSentPingResponses,
    kStatsValueNameRecvPingRequests,
    kStatsValueNameRecvPingResponses,
    kStatsValueNameSentStunKeepaliveRequests,
    kStatsValueNameRecvStunKeepaliveResponses,
    kStatsValueNameStunKeepaliveRttTotal,
    kStatsValueNameStunKeepaliveRttSquaredTotal,

    // Internal StatsValue names.
    kStatsValueNameAccelerateRate,
    kStatsValueNameActualEncBitrate,
    kStatsValueNameAdaptationChanges,
    kStatsValueNameAvailableReceiveBandwidth,
    kStatsValueNameAvailableSendBandwidth,
    kStatsValueNameAvgEncodeMs,
    kStatsValueNameBandwidthLimitedResolution,
    kStatsValueNameBucketDelay,
    kStatsValueNameCaptureStartNtpTimeMs,
    kStatsValueNameCandidateIPAddress,
    kStatsValueNameCandidateNetworkType,
    kStatsValueNameCandidatePortNumber,
    kStatsValueNameCandidatePriority,
    kStatsValueNameCandidateTransportType,
    kStatsValueNameCandidateType,
    kStatsValueNameChannelId,
    kStatsValueNameCodecName,
    kStatsValueNameComponent,
    kStatsValueNameContentName,
    kStatsValueNameContentType,
    kStatsValueNameCpuLimitedResolution,
    kStatsValueNameCurrentDelayMs,
    kStatsValueNameDecodeMs,
    kStatsValueNameDecodingCNG,
    kStatsValueNameDecodingCTN,
    kStatsValueNameDecodingCTSG,
    kStatsValueNameDecodingMutedOutput,
    kStatsValueNameDecodingNormal,
    kStatsValueNameDecodingPLC,
    kStatsValueNameDecodingCodecPLC,
    kStatsValueNameDecodingPLCCNG,
    kStatsValueNameDer,
    kStatsValueNameDtlsCipher,
    kStatsValueNameEchoDelayMedian,
    kStatsValueNameEchoDelayStdDev,
    kStatsValueNameEchoReturnLoss,
    kStatsValueNameEchoReturnLossEnhancement,
    kStatsValueNameEncodeUsagePercent,
    kStatsValueNameExpandRate,
    kStatsValueNameFingerprint,
    kStatsValueNameFingerprintAlgorithm,
    kStatsValueNameFirsReceived,
    kStatsValueNameFirsSent,
    kStatsValueNameFirstFrameReceivedToDecodedMs,
    kStatsValueNameFrameHeightInput,
    kStatsValueNameFrameHeightReceived,
    kStatsValueNameFrameHeightSent,
    kStatsValueNameFrameRateDecoded,
    kStatsValueNameFrameRateInput,
    kStatsValueNameFrameRateOutput,
    kStatsValueNameFrameRateReceived,
    kStatsValueNameFrameRateSent,
    kStatsValueNameFrameWidthInput,
    kStatsValueNameFrameWidthReceived,
    kStatsValueNameFrameWidthSent,
    kStatsValueNameHasEnteredLowResolution,
    kStatsValueNameHugeFramesSent,
    kStatsValueNameInitiator,
    kStatsValueNameInterframeDelayMaxMs,  // Max over last 10 seconds.
    kStatsValueNameIssuerId,
    kStatsValueNameJitterBufferMs,
    kStatsValueNameJitterReceived,
    kStatsValueNameLabel,
    kStatsValueNameLocalAddress,
    kStatsValueNameLocalCandidateId,
    kStatsValueNameLocalCandidateType,
    kStatsValueNameLocalCertificateId,
    kStatsValueNameMaxDecodeMs,
    kStatsValueNameMinPlayoutDelayMs,
    kStatsValueNameNacksReceived,
    kStatsValueNameNacksSent,
    kStatsValueNamePlisReceived,
    kStatsValueNamePlisSent,
    kStatsValueNamePreemptiveExpandRate,
    kStatsValueNamePreferredJitterBufferMs,
    kStatsValueNameRemoteAddress,
    kStatsValueNameRemoteCandidateId,
    kStatsValueNameRemoteCandidateType,
    kStatsValueNameRemoteCertificateId,
    kStatsValueNameRenderDelayMs,
    kStatsValueNameResidualEchoLikelihood,
    kStatsValueNameResidualEchoLikelihoodRecentMax,
    kStatsValueNameAnaBitrateActionCounter,
    kStatsValueNameAnaChannelActionCounter,
    kStatsValueNameAnaDtxActionCounter,
    kStatsValueNameAnaFecActionCounter,
    kStatsValueNameAnaFrameLengthIncreaseCounter,
    kStatsValueNameAnaFrameLengthDecreaseCounter,
    kStatsValueNameAnaUplinkPacketLossFraction,
    kStatsValueNameRetransmitBitrate,
    kStatsValueNameRtt,
    kStatsValueNameSecondaryDecodedRate,
    kStatsValueNameSecondaryDiscardedRate,
    kStatsValueNameSendPacketsDiscarded,
    kStatsValueNameSpeechExpandRate,
    kStatsValueNameSrtpCipher,
    kStatsValueNameTargetDelayMs,
    kStatsValueNameTargetEncBitrate,
    kStatsValueNameTimingFrameInfo,  // Result of |TimingFrameInfo::ToString|
    kStatsValueNameTrackId,
    kStatsValueNameTransmitBitrate,
    kStatsValueNameTransportType,
    kStatsValueNameTypingNoiseState,
    kStatsValueNameWritable,
    kStatsValueNameAudioDeviceUnderrunCounter,
  };

  class RTC_EXPORT IdBase : public rtc::RefCountInterface {
   public:
    ~IdBase() override;
    StatsType type() const;

    // Users of IdBase will be using the Id typedef, which is compatible with
    // this Equals() function.  It simply calls the protected (and overridden)
    // Equals() method.
    bool Equals(const rtc::scoped_refptr<IdBase>& other) const {
      return Equals(*other.get());
    }

    virtual std::string ToString() const = 0;

   protected:
    // Protected since users of the IdBase type will be using the Id typedef.
    virtual bool Equals(const IdBase& other) const;

    explicit IdBase(StatsType type);  // Only meant for derived classes.
    const StatsType type_;

    static const char kSeparator = '_';
  };

  typedef rtc::scoped_refptr<IdBase> Id;

  struct RTC_EXPORT Value {
    enum Type {
      kInt,           // int.
      kInt64,         // int64_t.
      kFloat,         // float.
      kString,        // std::string
      kStaticString,  // const char*.
      kBool,          // bool.
      kId,            // Id.
    };

    Value(StatsValueName name, int64_t value, Type int_type);
    Value(StatsValueName name, float f);
    Value(StatsValueName name, const std::string& value);
    Value(StatsValueName name, const char* value);
    Value(StatsValueName name, bool b);
    Value(StatsValueName name, const Id& value);

    ~Value();

    // Support ref counting. Note that for performance reasons, we
    // don't use thread safe operations. Therefore, all operations
    // affecting the ref count (in practice, creation and copying of
    // the Values mapping) must occur on webrtc's signalling thread.
    int AddRef() const {
      RTC_DCHECK_RUN_ON(&thread_checker_);
      return ++ref_count_;
    }
    int Release() const {
      RTC_DCHECK_RUN_ON(&thread_checker_);
      int count = --ref_count_;
      if (!count)
        delete this;
      return count;
    }

    // TODO(tommi): This compares name as well as value...
    // I think we should only need to compare the value part and
    // move the name part into a hash map.
    bool Equals(const Value& other) const;

    // Comparison operators. Return true iff the current instance is of the
    // correct type and holds the same value.  No conversion is performed so
    // a string value of "123" is not equal to an int value of 123 and an int
    // value of 123 is not equal to a float value of 123.0f.
    // One exception to this is that types kInt and kInt64 can be compared and
    // kString and kStaticString too.
    bool operator==(const std::string& value) const;
    bool operator==(const char* value) const;
    bool operator==(int64_t value) const;
    bool operator==(bool value) const;
    bool operator==(float value) const;
    bool operator==(const Id& value) const;

    // Getters that allow getting the native value directly.
    // The caller must know the type beforehand or else hit a check.
    int int_val() const;
    int64_t int64_val() const;
    float float_val() const;
    const char* static_string_val() const;
    const std::string& string_val() const;
    bool bool_val() const;
    const Id& id_val() const;

    // Returns the string representation of |name|.
    const char* display_name() const;

    // Converts the native value to a string representation of the value.
    std::string ToString() const;

    Type type() const { return type_; }

    // TODO(tommi): Move |name| and |display_name| out of the Value struct.
    const StatsValueName name;

   private:
    SequenceChecker thread_checker_;
    mutable int ref_count_ RTC_GUARDED_BY(thread_checker_) = 0;

    const Type type_;
    // TODO(tommi): Use C++ 11 union and make value_ const.
    union InternalType {
      int int_;
      int64_t int64_;
      float float_;
      bool bool_;
      std::string* string_;
      const char* static_string_;
      Id* id_;
    } value_;

    RTC_DISALLOW_COPY_AND_ASSIGN(Value);
  };

  typedef rtc::scoped_refptr<Value> ValuePtr;
  typedef std::map<StatsValueName, ValuePtr> Values;

  // Ownership of |id| is passed to |this|.
  explicit StatsReport(const Id& id);
  ~StatsReport();

  // Factory functions for various types of stats IDs.
  static Id NewBandwidthEstimationId();
  static Id NewTypedId(StatsType type, const std::string& id);
  static Id NewTypedIntId(StatsType type, int id);
  static Id NewIdWithDirection(StatsType type,
                               const std::string& id,
                               Direction direction);
  static Id NewCandidateId(bool local, const std::string& id);
  static Id NewComponentId(const std::string& content_name, int component);
  static Id NewCandidatePairId(const std::string& content_name,
                               int component,
                               int index);

  const Id& id() const { return id_; }
  StatsType type() const { return id_->type(); }
  double timestamp() const { return timestamp_; }
  void set_timestamp(double t) { timestamp_ = t; }
  bool empty() const { return values_.empty(); }
  const Values& values() const { return values_; }

  const char* TypeToString() const;

  void AddString(StatsValueName name, const std::string& value);
  void AddString(StatsValueName name, const char* value);
  void AddInt64(StatsValueName name, int64_t value);
  void AddInt(StatsValueName name, int value);
  void AddFloat(StatsValueName name, float value);
  void AddBoolean(StatsValueName name, bool value);
  void AddId(StatsValueName name, const Id& value);

  const Value* FindValue(StatsValueName name) const;

 private:
  // The unique identifier for this object.
  // This is used as a key for this report in ordered containers,
  // so it must never be changed.
  const Id id_;
  double timestamp_;  // Time since 1970-01-01T00:00:00Z in milliseconds.
  Values values_;

  RTC_DISALLOW_COPY_AND_ASSIGN(StatsReport);
};

// Typedef for an array of const StatsReport pointers.
// Ownership of the pointers held by this implementation is assumed to lie
// elsewhere and lifetime guarantees are made by the implementation that uses
// this type.  In the StatsCollector, object ownership lies with the
// StatsCollection class.
typedef std::vector<const StatsReport*> StatsReports;

// A map from the report id to the report.
// This class wraps an STL container and provides a limited set of
// functionality in order to keep things simple.
class StatsCollection {
 public:
  StatsCollection();
  ~StatsCollection();

  typedef std::list<StatsReport*> Container;
  typedef Container::iterator iterator;
  typedef Container::const_iterator const_iterator;

  const_iterator begin() const;
  const_iterator end() const;
  size_t size() const;

  // Creates a new report object with |id| that does not already
  // exist in the list of reports.
  StatsReport* InsertNew(const StatsReport::Id& id);
  StatsReport* FindOrAddNew(const StatsReport::Id& id);
  StatsReport* ReplaceOrAddNew(const StatsReport::Id& id);

  // Looks for a report with the given |id|.  If one is not found, null
  // will be returned.
  StatsReport* Find(const StatsReport::Id& id);

 private:
  Container list_;
  SequenceChecker thread_checker_;
};

}  // namespace webrtc

#endif  // API_STATS_TYPES_H_
