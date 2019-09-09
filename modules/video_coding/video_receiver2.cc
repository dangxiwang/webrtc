/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <stddef.h>

#include <cstdint>
#include <vector>

#include "api/rtp_headers.h"
#include "api/video_codecs/video_codec.h"
#include "api/video_codecs/video_decoder.h"
#include "modules/include/module_common_types.h"
#include "modules/utility/include/process_thread.h"
#include "modules/video_coding/decoder_database.h"
#include "modules/video_coding/encoded_frame.h"
#include "modules/video_coding/generic_decoder.h"
#include "modules/video_coding/include/video_coding.h"
#include "modules/video_coding/include/video_coding_defines.h"
#include "modules/video_coding/internal_defines.h"
#include "modules/video_coding/jitter_buffer.h"
#include "modules/video_coding/media_opt_util.h"
#include "modules/video_coding/packet.h"
#include "modules/video_coding/receiver.h"
#include "modules/video_coding/timing.h"
#include "modules/video_coding/video_receiver2.h"
#include "rtc_base/checks.h"
#include "rtc_base/critical_section.h"
#include "rtc_base/location.h"
#include "rtc_base/logging.h"
#include "rtc_base/one_time_event.h"
#include "rtc_base/thread_checker.h"
#include "rtc_base/trace_event.h"
#include "system_wrappers/include/clock.h"

namespace webrtc {
namespace vcm {

VideoReceiver2::VideoReceiver2(Clock* clock, VCMTiming* timing)
    : clock_(clock),
      _timing(timing),
      _decodedFrameCallback(_timing, clock_),
      _codecDataBase() {
  decoder_thread_checker_.Detach();
}

VideoReceiver2::~VideoReceiver2() {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
}

// Register a receive callback. Will be called whenever there is a new frame
// ready for rendering.
int32_t VideoReceiver2::RegisterReceiveCallback(
    VCMReceiveCallback* receiveCallback) {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  RTC_DCHECK(!IsDecoderThreadRunning());
  // This value is set before the decoder thread starts and unset after
  // the decoder thread has been stopped.
  _decodedFrameCallback.SetUserReceiveCallback(receiveCallback);
  return VCM_OK;
}

// Register an externally defined decoder object.
void VideoReceiver2::RegisterExternalDecoder(VideoDecoder* externalDecoder,
                                             uint8_t payloadType) {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  RTC_DCHECK(!IsDecoderThreadRunning());
  if (externalDecoder == nullptr) {
    RTC_CHECK(_codecDataBase.DeregisterExternalDecoder(payloadType));
    return;
  }
  _codecDataBase.RegisterExternalDecoder(externalDecoder, payloadType);
}

void VideoReceiver2::TriggerDecoderShutdown() {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  RTC_DCHECK(IsDecoderThreadRunning());
}

void VideoReceiver2::DecoderThreadStarting() {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  RTC_DCHECK(!IsDecoderThreadRunning());
#if RTC_DCHECK_IS_ON
  decoder_thread_is_running_ = true;
#endif
}

void VideoReceiver2::DecoderThreadStopped() {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  RTC_DCHECK(IsDecoderThreadRunning());
#if RTC_DCHECK_IS_ON
  decoder_thread_is_running_ = false;
  decoder_thread_checker_.Detach();
#endif
}

// Must be called from inside the receive side critical section.
int32_t VideoReceiver2::Decode(const VCMEncodedFrame* frame) {
  RTC_DCHECK_RUN_ON(&decoder_thread_checker_);
  TRACE_EVENT0("webrtc", "VideoReceiver2::Decode");
  // Change decoder if payload type has changed
  VCMGenericDecoder* decoder =
      _codecDataBase.GetDecoder(*frame, &_decodedFrameCallback);
  if (decoder == nullptr) {
    return VCM_NO_CODEC_REGISTERED;
  }
  return decoder->Decode(*frame, clock_->TimeInMilliseconds());
}

// Register possible receive codecs, can be called multiple times
int32_t VideoReceiver2::RegisterReceiveCodec(const VideoCodec* receiveCodec,
                                             int32_t numberOfCores,
                                             bool requireKeyFrame) {
  RTC_DCHECK_RUN_ON(&construction_thread_checker_);
  RTC_DCHECK(!IsDecoderThreadRunning());
  if (receiveCodec == nullptr) {
    return VCM_PARAMETER_ERROR;
  }
  if (!_codecDataBase.RegisterReceiveCodec(receiveCodec, numberOfCores,
                                           requireKeyFrame)) {
    return -1;
  }
  return 0;
}

bool VideoReceiver2::IsDecoderThreadRunning() {
#if RTC_DCHECK_IS_ON
  return decoder_thread_is_running_;
#else
  return true;
#endif
}

}  // namespace vcm
}  // namespace webrtc
