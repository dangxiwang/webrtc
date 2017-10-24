/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

package org.webrtc.voiceengine;

import android.annotation.TargetApi;
import android.content.Context;
import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder.AudioSource;
import android.os.Process;
import java.lang.System;
import java.nio.ByteBuffer;
import java.util.concurrent.TimeUnit;
import org.webrtc.ContextUtils;
import org.webrtc.Logging;
import org.webrtc.ThreadUtils;

public class WebRtcAudioRecord {
  private static final boolean DEBUG = false;

  private static final String TAG = "WebRtcAudioRecord";

  // Default audio data format is PCM 16 bit per sample.
  // Guaranteed to be supported by all devices.
  private static final int BITS_PER_SAMPLE = 16;

  // Requested size of each recorded buffer provided to the client.
  private static final int CALLBACK_BUFFER_SIZE_MS = 10;

  // Average number of callbacks per second.
  private static final int BUFFERS_PER_SECOND = 1000 / CALLBACK_BUFFER_SIZE_MS;

  // We ask for a native buffer size of BUFFER_SIZE_FACTOR * (minimum required
  // buffer size). The extra space is allocated to guard against glitches under
  // high load.
  private static final int BUFFER_SIZE_FACTOR = 2;

  // The AudioRecordJavaThread is allowed to wait for successful call to join()
  // but the wait times out afther this amount of time.
  private static final long AUDIO_RECORD_THREAD_JOIN_TIMEOUT_MS = 2000;

  private final long nativeAudioRecord;
  private final ThreadUtils.ThreadChecker threadChecker = new ThreadUtils.ThreadChecker();

  private WebRtcAudioEffects effects = null;

  private ByteBuffer byteBuffer;

  private AudioRecord audioRecord = null;
  private AudioRecordThread audioThread = null;

  private static volatile boolean microphoneMute = false;
  private byte[] emptyBytes;

  // Audio recording error handler functions.
  public enum AudioRecordStartErrorCode {
    AUDIO_RECORD_START_EXCEPTION,
    AUDIO_RECORD_START_STATE_MISMATCH,
  }

  public static interface WebRtcAudioRecordErrorCallback {
    void onWebRtcAudioRecordInitError(String errorMessage);
    void onWebRtcAudioRecordStartError(AudioRecordStartErrorCode errorCode, String errorMessage);
    void onWebRtcAudioRecordError(String errorMessage);
  }

  private static WebRtcAudioRecordErrorCallback errorCallback = null;

  public static void setErrorCallback(WebRtcAudioRecordErrorCallback errorCallback) {
    Logging.d(TAG, "Set error callback");
    WebRtcAudioRecord.errorCallback = errorCallback;
  }

  /**
   * Audio thread which keeps calling ByteBuffer.read() waiting for audio
   * to be recorded. Feeds recorded data to the native counterpart as a
   * periodic sequence of callbacks using DataIsRecorded().
   * This thread uses a Process.THREAD_PRIORITY_URGENT_AUDIO priority.
   */
  private class AudioRecordThread extends Thread {
    private volatile boolean keepAlive = true;

    public AudioRecordThread(String name) {
      super(name);
    }

    @Override
    public void run() {
      Process.setThreadPriority(Process.THREAD_PRIORITY_URGENT_AUDIO);
      Logging.d(TAG, "AudioRecordThread" + WebRtcAudioUtils.getThreadInfo());
      assertTrue(audioRecord.getRecordingState() == AudioRecord.RECORDSTATE_RECORDING);

      long lastTime = System.nanoTime();
      long count = 0;
      while (keepAlive) {
        int bytesRead = audioRecord.read(byteBuffer, byteBuffer.capacity());
        if (bytesRead == byteBuffer.capacity()) {
          if (microphoneMute) {
            byteBuffer.clear();
            byteBuffer.put(emptyBytes);
          }
          // It's possible we've been shut down during the read, and stopRecording() tried and
          // failed to join this thread. To be a bit safer, try to avoid calling any native methods
          // in case they've been unregistered after stopRecording() returned.
          if (keepAlive) {
            nativeDataIsRecorded(bytesRead, nativeAudioRecord);
            count++;
          }
        } else {
          String errorMessage = "AudioRecord.read failed: " + bytesRead;
          Logging.e(TAG, errorMessage);
          if (bytesRead == AudioRecord.ERROR_INVALID_OPERATION) {
            keepAlive = false;
            reportWebRtcAudioRecordError(errorMessage);
          }
        }

        // Wait approx. one second.
        if (count == 100) {
          Logging.d(TAG, "Restarting the AEC....");
          effects.restartAEC();
        }

        if (DEBUG) {
          long nowTime = System.nanoTime();
          long durationInMs = TimeUnit.NANOSECONDS.toMillis((nowTime - lastTime));
          lastTime = nowTime;
          Logging.d(TAG, "bytesRead[" + durationInMs + "] " + bytesRead);
        }
      }

      try {
        if (audioRecord != null) {
          audioRecord.stop();
        }
      } catch (IllegalStateException e) {
        Logging.e(TAG, "AudioRecord.stop failed: " + e.getMessage());
      }
    }

    // Stops the inner thread loop and also calls AudioRecord.stop().
    // Does not block the calling thread.
    public void stopThread() {
      Logging.d(TAG, "stopThread");
      keepAlive = false;
    }
  }

  WebRtcAudioRecord(long nativeAudioRecord) {
    threadChecker.checkIsOnValidThread();
    Logging.d(TAG, "ctor" + WebRtcAudioUtils.getThreadInfo());
    this.nativeAudioRecord = nativeAudioRecord;
    if (DEBUG) {
      WebRtcAudioUtils.logDeviceInfo(TAG);
    }
    effects = WebRtcAudioEffects.create();
  }

  private boolean enableBuiltInAEC(boolean enable) {
    threadChecker.checkIsOnValidThread();
    Logging.d(TAG, "enableBuiltInAEC(" + enable + ')');
    if (effects == null) {
      Logging.e(TAG, "Built-in AEC is not supported on this platform");
      return false;
    }
    return effects.setAEC(enable);
  }

  private boolean enableBuiltInNS(boolean enable) {
    threadChecker.checkIsOnValidThread();
    Logging.d(TAG, "enableBuiltInNS(" + enable + ')');
    if (effects == null) {
      Logging.e(TAG, "Built-in NS is not supported on this platform");
      return false;
    }
    return effects.setNS(enable);
  }

  private int initRecording(int sampleRate, int channels) {
    threadChecker.checkIsOnValidThread();
    Logging.d(TAG, "initRecording(sampleRate=" + sampleRate + ", channels=" + channels + ")");
    if (!WebRtcAudioUtils.hasPermission(
            ContextUtils.getApplicationContext(), android.Manifest.permission.RECORD_AUDIO)) {
      reportWebRtcAudioRecordInitError("RECORD_AUDIO permission is missing");
      return -1;
    }
    if (audioRecord != null) {
      reportWebRtcAudioRecordInitError("InitRecording called twice without StopRecording.");
      return -1;
    }
    final int bytesPerFrame = channels * (BITS_PER_SAMPLE / 8);
    final int framesPerBuffer = sampleRate / BUFFERS_PER_SECOND;
    byteBuffer = ByteBuffer.allocateDirect(bytesPerFrame * framesPerBuffer);
    Logging.d(TAG, "byteBuffer.capacity: " + byteBuffer.capacity());
    emptyBytes = new byte[byteBuffer.capacity()];
    // Rather than passing the ByteBuffer with every callback (requiring
    // the potentially expensive GetDirectBufferAddress) we simply have the
    // the native class cache the address to the memory once.
    nativeCacheDirectBufferAddress(byteBuffer, nativeAudioRecord);

    // Get the minimum buffer size required for the successful creation of
    // an AudioRecord object, in byte units.
    // Note that this size doesn't guarantee a smooth recording under load.
    final int channelConfig = channelCountToConfiguration(channels);
    int minBufferSize =
        AudioRecord.getMinBufferSize(sampleRate, channelConfig, AudioFormat.ENCODING_PCM_16BIT);
    if (minBufferSize == AudioRecord.ERROR || minBufferSize == AudioRecord.ERROR_BAD_VALUE) {
      reportWebRtcAudioRecordInitError("AudioRecord.getMinBufferSize failed: " + minBufferSize);
      return -1;
    }
    Logging.d(TAG, "AudioRecord.getMinBufferSize: " + minBufferSize);

    // Use a larger buffer size than the minimum required when creating the
    // AudioRecord instance to ensure smooth recording under load. It has been
    // verified that it does not increase the actual recording latency.
    int bufferSizeInBytes = Math.max(BUFFER_SIZE_FACTOR * minBufferSize, byteBuffer.capacity());
    Logging.d(TAG, "bufferSizeInBytes: " + bufferSizeInBytes);
    try {
      if (WebRtcAudioUtils.runningOnMarshmallowOrHigher()) {
        // Use AudioRecord.Builder to create the AudioRecord instance if we are on API level 23 or
        // higher.
        audioRecord = createAudioRecordOnMarshmallowOrHigher(
            sampleRate, channelConfig, bufferSizeInBytes);
      } else {
        // Use default constructor for API levels below 23.
        audioRecord = new AudioRecord(AudioSource.VOICE_COMMUNICATION, sampleRate,
            channelConfig, AudioFormat.ENCODING_PCM_16BIT, bufferSizeInBytes);
      }
    } catch (IllegalArgumentException e) {
      reportWebRtcAudioRecordInitError("AudioRecord ctor error: " + e.getMessage());
      releaseAudioResources();
      return -1;
    }
    if (audioRecord == null || audioRecord.getState() != AudioRecord.STATE_INITIALIZED) {
      reportWebRtcAudioRecordInitError("Failed to create a new AudioRecord instance");
      releaseAudioResources();
      return -1;
    }
    if (effects != null) {
      effects.enable(audioRecord.getAudioSessionId());
    }
    logMainParameters();
    logMainParametersExtended();
    return framesPerBuffer;
  }

  private boolean startRecording() {
    threadChecker.checkIsOnValidThread();
    Logging.d(TAG, "startRecording");
    assertTrue(audioRecord != null);
    assertTrue(audioThread == null);

    // Starts recording from the AudioRecord instance.
    try {
      audioRecord.startRecording();
    } catch (IllegalStateException e) {
      reportWebRtcAudioRecordStartError(AudioRecordStartErrorCode.AUDIO_RECORD_START_EXCEPTION,
          "AudioRecord.startRecording failed: " + e.getMessage());
      return false;
    }

    // Verify the recording state up to two times (with a sleep in between)
    // before returning false and reporting an error.
    int numberOfStateChecks = 0;
    while (audioRecord.getRecordingState() != AudioRecord.RECORDSTATE_RECORDING &&
           ++numberOfStateChecks < 2) {
      threadSleep(200);
    }
    if (audioRecord.getRecordingState() != AudioRecord.RECORDSTATE_RECORDING) {
      reportWebRtcAudioRecordStartError(
          AudioRecordStartErrorCode.AUDIO_RECORD_START_STATE_MISMATCH,
          "AudioRecord.startRecording failed - incorrect state :"
          + audioRecord.getRecordingState());
      return false;
    }

    // Create and start new high-priority thread which calls AudioRecord.read()
    // and where we also call the native DataIsRecorded() callback to feed
    // WebRTC with recorded audio.
    audioThread = new AudioRecordThread("AudioRecordJavaThread");
    audioThread.start();
    return true;
  }

  private boolean stopRecording() {
    threadChecker.checkIsOnValidThread();
    Logging.d(TAG, "stopRecording");
    assertTrue(audioThread != null);
    audioThread.stopThread();
    if (!ThreadUtils.joinUninterruptibly(audioThread, AUDIO_RECORD_THREAD_JOIN_TIMEOUT_MS)) {
      Logging.e(TAG, "Join of AudioRecordJavaThread timed out");
    }
    audioThread = null;
    if (effects != null) {
      effects.release();
    }
    releaseAudioResources();
    return true;
  }

  private void logMainParameters() {
    Logging.d(TAG, "AudioRecord: "
            + "session ID: " + audioRecord.getAudioSessionId() + ", "
            + "channels: " + audioRecord.getChannelCount() + ", "
            + "sample rate: " + audioRecord.getSampleRate());
  }

  @TargetApi(23)
  private void logMainParametersExtended() {
    if (WebRtcAudioUtils.runningOnMarshmallowOrHigher()) {
      Logging.d(TAG, "AudioRecord: "
              // The frame count of the native AudioRecord buffer.
              + "buffer size in frames: " + audioRecord.getBufferSizeInFrames());
    }
  }

  // Creates an AudioRecord instance using AudioRecord.Builder which was added in API level 23.
  @TargetApi(23)
  private AudioRecord createAudioRecordOnMarshmallowOrHigher(
    int sampleRateInHz, int channelConfig, int bufferSizeInBytes) {
    Logging.d(TAG, "createAudioRecordOnMarshmallowOrHigher");
    return new AudioRecord.Builder()
        .setAudioSource(AudioSource.VOICE_COMMUNICATION)
        .setAudioFormat(new AudioFormat.Builder()
            .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
            .setSampleRate(sampleRateInHz)
            .setChannelMask(channelConfig)
            .build())
        .setBufferSizeInBytes(bufferSizeInBytes)
        .build();
  }

  // Helper method which throws an exception  when an assertion has failed.
  private static void assertTrue(boolean condition) {
    if (!condition) {
      throw new AssertionError("Expected condition to be true");
    }
  }

  private int channelCountToConfiguration(int channels) {
    return (channels == 1 ? AudioFormat.CHANNEL_IN_MONO : AudioFormat.CHANNEL_IN_STEREO);
  }

  private native void nativeCacheDirectBufferAddress(ByteBuffer byteBuffer, long nativeAudioRecord);

  private native void nativeDataIsRecorded(int bytes, long nativeAudioRecord);

  // Sets all recorded samples to zero if |mute| is true, i.e., ensures that
  // the microphone is muted.
  public static void setMicrophoneMute(boolean mute) {
    Logging.w(TAG, "setMicrophoneMute(" + mute + ")");
    microphoneMute = mute;
  }

  // Releases the native AudioRecord resources.
  private void releaseAudioResources() {
    if (audioRecord != null) {
      audioRecord.release();
      audioRecord = null;
    }
  }

  private void reportWebRtcAudioRecordInitError(String errorMessage) {
    Logging.e(TAG, "Init recording error: " + errorMessage);
    if (errorCallback != null) {
      errorCallback.onWebRtcAudioRecordInitError(errorMessage);
    }
  }

  private void reportWebRtcAudioRecordStartError(
      AudioRecordStartErrorCode errorCode, String errorMessage) {
    Logging.e(TAG, "Start recording error: " + errorCode + ". " + errorMessage);
    if (errorCallback != null) {
      errorCallback.onWebRtcAudioRecordStartError(errorCode, errorMessage);
    }
  }

  private void reportWebRtcAudioRecordError(String errorMessage) {
    Logging.e(TAG, "Run-time recording error: " + errorMessage);
    if (errorCallback != null) {
      errorCallback.onWebRtcAudioRecordError(errorMessage);
    }
  }

  // Causes the currently executing thread to sleep for the specified number
  // of milliseconds.
  private void threadSleep(long millis) {
    try {
      Thread.sleep(millis);
    } catch (InterruptedException e) {
      Logging.e(TAG, "Thread.sleep failed: " + e.getMessage());
    }
  }
}
