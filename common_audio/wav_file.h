/*
 *  Copyright (c) 2014 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef COMMON_AUDIO_WAV_FILE_H_
#define COMMON_AUDIO_WAV_FILE_H_

#include <stdint.h>

#include <cstddef>
#include <string>

#include "common_audio/wav_header.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/system/file_wrapper.h"

namespace webrtc {

// Interface to provide access to WAV file parameters.
class WavFile {
 public:
  enum class SampleFormats { kInt16, kFloat };

  virtual ~WavFile() {}

  virtual int sample_rate() const = 0;
  virtual size_t num_channels() const = 0;
  virtual size_t num_samples() const = 0;
};

// Simple C++ class for writing 16-bit PCM WAV files. All error handling is
// by calls to RTC_CHECK(), making it unsuitable for anything but debug code.
class WavWriter final : public WavFile {
 public:
  // Open a new WAV file for writing.
  WavWriter(const std::string& filename,
            int sample_rate,
            size_t num_channels,
            SampleFormats sample_format = SampleFormats::kInt16);

  // Open a new WAV file for writing.
  WavWriter(FileWrapper file,
            int sample_rate,
            size_t num_channels,
            SampleFormats sample_format = SampleFormats::kInt16);

  // Close the WAV file, after writing its header.
  ~WavWriter() override { Close(); }

  // Write additional samples to the file. Each sample is in the range
  // [-32768.0,32767.0], and there must be the previously specified number of
  // interleaved channels.
  void WriteSamples(const float* samples, size_t num_samples);
  void WriteSamples(const int16_t* samples, size_t num_samples);

  int sample_rate() const override { return sample_rate_; }
  size_t num_channels() const override { return num_channels_; }
  size_t num_samples() const override { return num_samples_; }

 private:
  void Close();
  const int sample_rate_;
  const size_t num_channels_;
  size_t num_samples_;  // Total number of samples written to file.
  WavFormat format_;    // The WAV file format used.
  FileWrapper file_;    // Output file, owned by this class.
  // const SampleFormats sample_format_;

  RTC_DISALLOW_COPY_AND_ASSIGN(WavWriter);
};

// Follows the conventions of WavWriter.
class WavReader final : public WavFile {
 public:
  // Opens an existing WAV file for reading.
  explicit WavReader(const std::string& filename);

  // Use an existing WAV file for reading.
  explicit WavReader(FileWrapper file);

  // Close the WAV file.
  ~WavReader() override { Close(); }

  // Resets position to the beginning of the file.
  void Reset();

  // Returns the number of samples read. If this is less than requested,
  // verifies that the end of the file was reached.
  size_t ReadSamples(size_t num_samples, float* samples);
  size_t ReadSamples(size_t num_samples, int16_t* samples);

  int sample_rate() const override { return sample_rate_; }
  size_t num_channels() const override { return num_channels_; }
  size_t num_samples() const override { return num_samples_; }

 private:
  void Close();
  int sample_rate_;
  size_t num_channels_;
  size_t num_samples_;  // Total number of samples in the file.
  WavFormat format_;    // The WAV file format used.
  size_t num_samples_remaining_;
  FileWrapper file_;  // Input file, owned by this class.
  int64_t
      data_start_pos_;  // Position in the file immediately after WAV header.
  // SampleFormats sample_format_;

  RTC_DISALLOW_COPY_AND_ASSIGN(WavReader);
};

}  // namespace webrtc

#endif  // COMMON_AUDIO_WAV_FILE_H_
