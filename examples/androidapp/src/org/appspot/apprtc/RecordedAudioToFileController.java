/*
 *  Copyright 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

package org.appspot.apprtc;

import android.media.AudioFormat;
import android.os.Environment;
import android.util.Log;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.OutputStream;
import java.util.concurrent.ExecutorService;
import org.webrtc.voiceengine.WebRtcAudioRecord;
import org.webrtc.voiceengine.WebRtcAudioRecord.AudioSamples;
import org.webrtc.voiceengine.WebRtcAudioRecord.WebRtcAudioRecordSamplesReadyCallback;

/**
 * Implements the WebRtcAudioRecordSamplesReadyCallback interface and writes
 * recorded raw audio samples to an output file.
 */
public class RecordedAudioToFileController
    implements WebRtcAudioRecordSamplesReadyCallback {
  private static final String TAG = "RecordedAudioToFile";
  private static final long MAX_FILE_SIZE_IN_BYTES = 58348800L;

  private final ExecutorService executor;
  private OutputStream rawAudioFileOutputStream = null;
  private long fileSizeInBytes = 0;

  public RecordedAudioToFileController(ExecutorService executor) {
    Log.d(TAG, "ctor");
    this.executor = executor;
  }

  public boolean start() {
    Log.d(TAG, "start");
    if (!isExternalStorageWritable()) {
      Log.e(TAG, "Writing to external media is not possible");
      return false;
    }
    // Register this class as receiver of recorded audio samples for storage.
    WebRtcAudioRecord.setOnAudioSamplesReady(this);
    return true;
  }

  public void stop() {
    Log.d(TAG, "stop");
    // De-register this class as receiver of recorded audio samples for storage.
    WebRtcAudioRecord.setOnAudioSamplesReady(null);
    if (rawAudioFileOutputStream != null) {
      try {
        rawAudioFileOutputStream.close();
      } catch (IOException e) {
        Log.e(TAG, "Failed to close file with saved input audio: " + e);
      }
      rawAudioFileOutputStream = null;
    }
    fileSizeInBytes = 0;
  }

  // Checks if external storage is available for read and write.
  private boolean isExternalStorageWritable() {
    String state = Environment.getExternalStorageState();
    if (Environment.MEDIA_MOUNTED.equals(state)) {
        return true;
    }
    return false;
  }

  // Utilizes audio parameters to create a file name which contains sufficient
  // information so that the file can be played using an external file player.
  // Example: /sdcard/recorded_audio_16bits_48000Hz_mono.pcm.
  // private void openRawAudioOutputFile(int sampleRate, int channelCount) {
  private void openRawAudioOutputFile(int sampleRate, int channelCount) {
    final String fileName = Environment.getExternalStorageDirectory().getPath()
        + File.separator + "recorded_audio_16bits_"
        + String.valueOf(sampleRate) + "Hz"
        + ((channelCount == 1) ? "_mono" : "_stereo") + ".pcm";
    final File outputFile = new File(fileName);
    try {
      rawAudioFileOutputStream = new FileOutputStream(outputFile);
    } catch (FileNotFoundException e) {
      Log.e(TAG, "Failed to open audio output file: " + e.getMessage());
    }
    Log.d(TAG, "Opened file for recording: " + fileName);
  }

  @Override
  // Called when new audio samples are ready.
  public void onWebRtcAudioRecordSamplesReady(AudioSamples samples) {
    // The native audio layer on Android should use 16-bit PCM format.
    if (samples.getAudioFormat() != AudioFormat.ENCODING_PCM_16BIT) {
      Log.e(TAG, "Invalid audio format");
      return;
    }
    // Open a new file for the first callback only since it allows us to add
    // audio parameters to the file name.
    if (rawAudioFileOutputStream == null) {
      openRawAudioOutputFile(samples.getSampleRate(), samples.getChannelCount());
      fileSizeInBytes = 0;
    }
    // Append the recorded 16-bit audio samples to the open output file.
    executor.execute(new Runnable() {
      @Override
      public void run() {
        if (rawAudioFileOutputStream != null) {
          try {
            // Set a limit on max file size. 58348800 bytes corresponds to
            // approximately 10 minutes of recording in mono at 48kHz.
            if (fileSizeInBytes < MAX_FILE_SIZE_IN_BYTES) {
              // Writes samples.getData().length bytes to output stream.
              rawAudioFileOutputStream.write(samples.getData());
              fileSizeInBytes += samples.getData().length;
            }
          } catch (IOException e) {
            Log.e(TAG, "Failed to write audio to file: " + e.getMessage());
          }
        }
      }
    });
  }
}
