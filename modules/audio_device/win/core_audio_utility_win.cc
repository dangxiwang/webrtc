/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_device/win/core_audio_utility_win.h"

#include <Functiondiscoverykeys_devpkey.h>
#include <comdef.h>
#include <stdio.h>
#include <tchar.h>

#include <iomanip>
#include <string>

#include "rtc_base/arraysize.h"
#include "rtc_base/logging.h"
#include "rtc_base/platform_thread_types.h"
#include "rtc_base/strings/string_builder.h"
#include "rtc_base/stringutils.h"
#include "rtc_base/timedelta.h"

using Microsoft::WRL::ComPtr;

namespace webrtc {
namespace win {

namespace {

std::string WaveFormatToString(const WAVEFORMATPCMEX* format) {
  char ss_buf[1024];
  rtc::SimpleStringBuilder ss(ss_buf);
  ss.AppendFormat("wFormatTag: 0x%X", format->Format.wFormatTag);
  ss.AppendFormat(", nChannels: %d", format->Format.nChannels);
  ss.AppendFormat(", nSamplesPerSec: %d", format->Format.nSamplesPerSec);
  ss.AppendFormat(", nAvgBytesPerSec: %d", format->Format.nAvgBytesPerSec);
  ss.AppendFormat(", nBlockAlign: %d", format->Format.nBlockAlign);
  ss.AppendFormat(", wBitsPerSample: %d", format->Format.wBitsPerSample);
  ss.AppendFormat(", cbSize: %d", format->Format.cbSize);
  ss.AppendFormat(", wValidBitsPerSample: %d",
                  format->Samples.wValidBitsPerSample);
  ss.AppendFormat(", dwChannelMask: 0x%X", format->dwChannelMask);
  return ss.str();
}

std::string ErrorToString(const _com_error& error) {
  char ss_buf[1024];
  rtc::SimpleStringBuilder ss(ss_buf);
  ss.AppendFormat("%s (0x%08X)", rtc::ToUtf8(error.ErrorMessage()).c_str(),
                  error.Error());
  return ss.str();
}

rtc::TimeDelta ReferenceTimeToTimeDelta(REFERENCE_TIME time) {
  // Each unit of reference time is 100 nanoseconds <=> 0.1 microsecond.
  return rtc::TimeDelta::FromMicroseconds(0.1 * time + 0.5);
}

bool LoadAudiosesDll() {
  static const wchar_t* const kAudiosesDLL =
      L"%WINDIR%\\system32\\audioses.dll";
  wchar_t path[MAX_PATH] = {0};
  ExpandEnvironmentStringsW(kAudiosesDLL, path, arraysize(path));
  // RTC_DLOG(INFO) << rtc::ToUtf8(path);
  return (LoadLibraryExW(path, NULL, LOAD_WITH_ALTERED_SEARCH_PATH) != NULL);
}

ComPtr<IMMDeviceEnumerator> CreateDeviceEnumeratorInternal(
    bool allow_reinitialize) {
  ComPtr<IMMDeviceEnumerator> device_enumerator;
  _com_error error = ::CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL,
                                        CLSCTX_INPROC_SERVER,
                                        IID_PPV_ARGS(&device_enumerator));

  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "CoCreateInstance failed: " << ErrorToString(error);
  }

  if (error.Error() == CO_E_NOTINITIALIZED && allow_reinitialize) {
    RTC_LOG(LS_ERROR) << "CoCreateInstance failed with CO_E_NOTINITIALIZED";
    // We have seen crashes which indicates that this method can in fact
    // fail with CO_E_NOTINITIALIZED in combination with certain 3rd party
    // modules. Calling CoInitializeEx() is an attempt to resolve the reported
    // issues. See http://crbug.com/378465 for details.
    error = CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if (error.Error() != S_OK) {
      error = ::CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL,
                                 CLSCTX_INPROC_SERVER,
                                 IID_PPV_ARGS(&device_enumerator));
      if (error.Error() != S_OK) {
        RTC_LOG(LS_ERROR) << "CoCreateInstance failed: "
                          << ErrorToString(error);
      }
    }
  }
  return device_enumerator;
}

bool IsSupportedInternal() {
  // The Core Audio APIs are implemented in the user-mode system components
  // Audioses.dll and Mmdevapi.dll. Dependency Walker shows that it is
  // enough to verify possibility to load the Audioses DLL since it depends
  // on Mmdevapi.dll. See http://crbug.com/166397 why this extra step is
  // required to guarantee Core Audio support.
  if (!LoadAudiosesDll())
    return false;

  // Being able to load the Audioses.dll does not seem to be sufficient for
  // all devices to guarantee Core Audio support. To be 100%, we also verify
  // that it is possible to a create the IMMDeviceEnumerator interface. If
  // this works as well we should be home free.
  ComPtr<IMMDeviceEnumerator> device_enumerator =
      CreateDeviceEnumeratorInternal(false);
  if (!device_enumerator) {
    RTC_LOG(LS_ERROR)
        << "Failed to create Core Audio device enumerator on thread with ID "
        << rtc::CurrentThreadId();
    return false;
  }

  return true;
}

bool IsDeviceActive(IMMDevice* device) {
  DWORD state = DEVICE_STATE_DISABLED;
  return SUCCEEDED(device->GetState(&state)) && (state & DEVICE_STATE_ACTIVE);
}

// Retrieve an audio device specified by |device_id| or a default device
// specified by data-flow direction and role if |device_id| is default.
ComPtr<IMMDevice> CreateDeviceInternal(const std::string& device_id,
                                       EDataFlow data_flow,
                                       ERole role) {
  ComPtr<IMMDevice> audio_endpoint_device;

  // Create the IMMDeviceEnumerator interface.
  ComPtr<IMMDeviceEnumerator> device_enum(CreateDeviceEnumeratorInternal(true));
  if (!device_enum.Get())
    return audio_endpoint_device;

  _com_error error(S_FALSE);
  if (device_id == AudioDeviceName::kDefaultDeviceId) {
    error = device_enum->GetDefaultAudioEndpoint(
        data_flow, role, audio_endpoint_device.GetAddressOf());
    if (error.Error() != S_OK) {
      RTC_LOG(LS_ERROR)
          << "IMMDeviceEnumerator::GetDefaultAudioEndpoint failed: "
          << ErrorToString(error);
    }
  } else {
    error = device_enum->GetDevice(rtc::ToUtf16(device_id).c_str(),
                                   audio_endpoint_device.GetAddressOf());
    if (error.Error() != S_OK) {
      RTC_LOG(LS_ERROR) << "IMMDeviceEnumerator::GetDevice failed: "
                        << ErrorToString(error);
    }
  }

  // Verify that the audio endpoint device is active, i.e., that the audio
  // adapter that connects to the endpoint device is present and enabled.
  if (SUCCEEDED(error.Error()) &&
      !IsDeviceActive(audio_endpoint_device.Get())) {
    RTC_LOG(LS_WARNING) << "Selected endpoint device is not active";
    audio_endpoint_device.Reset();
  }

  return audio_endpoint_device;
}

std::string GetDeviceIdInternal(IMMDevice* device) {
  // Retrieve unique name of endpoint device.
  // Example: "{0.0.1.00000000}.{8db6020f-18e3-4f25-b6f5-7726c9122574}".
  ScopedCoMem<WCHAR> device_id;
  if (SUCCEEDED(device->GetId(&device_id))) {
    return rtc::ToUtf8(device_id, wcslen(device_id));
  } else {
    return std::string();
  }
}

std::string GetDeviceFriendlyNameInternal(IMMDevice* device) {
  // Retrieve user-friendly name of endpoint device.
  // Example: "Microphone (Realtek High Definition Audio)".
  ComPtr<IPropertyStore> properties;
  HRESULT hr = device->OpenPropertyStore(STGM_READ, properties.GetAddressOf());
  if (FAILED(hr))
    return std::string();

  ScopedPropVariant friendly_name_pv;
  hr = properties->GetValue(PKEY_Device_FriendlyName,
                            friendly_name_pv.Receive());
  if (FAILED(hr))
    return std::string();

  if (friendly_name_pv.get().vt == VT_LPWSTR &&
      friendly_name_pv.get().pwszVal) {
    return rtc::ToUtf8(friendly_name_pv.get().pwszVal,
                       wcslen(friendly_name_pv.get().pwszVal));
  } else {
    return std::string();
  }
}

// Creates and activates an IAudioClient COM object given the selected
// endpoint device.
ComPtr<IAudioClient> CreateClientInternal(IMMDevice* audio_device) {
  if (!audio_device)
    return ComPtr<IAudioClient>();

  ComPtr<IAudioClient> audio_client;
  _com_error error = audio_device->Activate(
      __uuidof(IAudioClient), CLSCTX_INPROC_SERVER, NULL, &audio_client);
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IMMDevice::Activate(IAudioClient) failed: "
                      << ErrorToString(error);
  }
  return audio_client;
}

ComPtr<IMMDeviceCollection> CreateCollectionInternal(EDataFlow data_flow) {
  ComPtr<IMMDeviceEnumerator> device_enumerator(
      CreateDeviceEnumeratorInternal(true));
  if (!device_enumerator) {
    return ComPtr<IMMDeviceCollection>();
  }

  // Generate a collection of active (present and not disabled) audio endpoint
  // devices for the specified data-flow direction.
  // This method will succeed even if all devices are disabled.
  ComPtr<IMMDeviceCollection> collection;
  _com_error error = device_enumerator->EnumAudioEndpoints(
      data_flow, DEVICE_STATE_ACTIVE, collection.GetAddressOf());
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IMMDeviceCollection::EnumAudioEndpoints failed: "
                      << ErrorToString(error);
  }
  return collection;
}

bool GetDeviceNamesInternal(EDataFlow data_flow,
                            AudioDeviceNames* device_names) {
  // Always add the default device in index 0 and the default communication
  // device as index 1 in the vector. The name of the default device starts
  // with "Default - " and the default communication device starts with
  // "Communication - ".
  //  Example of friendly name: "Default - Headset (SB Arena Headset)"
  ERole role[] = {eConsole, eCommunications};
  ComPtr<IMMDevice> default_device;
  AudioDeviceName default_device_name;
  for (size_t i = 0; i < arraysize(role); ++i) {
    default_device = CreateDeviceInternal(AudioDeviceName::kDefaultDeviceId,
                                          data_flow, role[i]);
    std::string device_name;
    device_name += (role[i] == eConsole ? "Default - " : "Communication - ");
    device_name += GetDeviceFriendlyNameInternal(default_device.Get());
    std::string unique_id = GetDeviceIdInternal(default_device.Get());

    default_device_name.SetName(device_name);
    default_device_name.SetId(unique_id);
    RTC_DLOG(INFO) << "friendly name: " << default_device_name.device_name;
    RTC_DLOG(INFO) << "unique id    : " << default_device_name.unique_id;
    // Add combination of user-friendly and unique name to the output list.
    device_names->emplace_back(default_device_name);
  }

  // Next, add all active input devices on index 2 and above. Note that,
  // one device can have more than one role. Hence, if only one input device
  // is present, the output vector will contain three elements all with the
  // same unique ID but with different names.
  // Example (one capture device but three elements in device_names):
  //   0: friendly name: Default - Headset (SB Arena Headset)
  //   0: unique id    : {0.0.1.00000000}.{822d99bb-d9b0-4f6f-b2a5-cd1be220d338}
  //   1: friendly name: Communication - Headset (SB Arena Headset)
  //   1: unique id    : {0.0.1.00000000}.{822d99bb-d9b0-4f6f-b2a5-cd1be220d338}
  //   2: friendly name: Headset (SB Arena Headset)
  //   2: unique id    : {0.0.1.00000000}.{822d99bb-d9b0-4f6f-b2a5-cd1be220d338}

  // Generate a collection of active audio endpoint devices for the specified
  // direction.
  ComPtr<IMMDeviceCollection> collection = CreateCollectionInternal(data_flow);
  if (!collection.Get()) {
    return false;
  }

  // Retrieve the number of active audio devices for the specified direction.
  UINT number_of_active_devices = 0;
  collection->GetCount(&number_of_active_devices);
  if (number_of_active_devices == 0) {
    return true;
  }

  // Loop over all active devices and add friendly name and unique ID to the
  // |device_names| list which already contains two elements
  RTC_DCHECK_EQ(device_names->size(), 2);
  for (UINT i = 0; i < number_of_active_devices; ++i) {
    // Retrieve a pointer to the specified item in the device collection.
    ComPtr<IMMDevice> audio_device;
    _com_error error = collection->Item(i, audio_device.GetAddressOf());
    if (error.Error() != S_OK)
      continue;
    // Retrieve the complete device name for the given audio device endpoint.
    AudioDeviceName device_name(
        GetDeviceFriendlyNameInternal(audio_device.Get()),
        GetDeviceIdInternal(audio_device.Get()));
    RTC_DLOG(INFO) << "friendly name: " << device_name.device_name;
    RTC_DLOG(INFO) << "unique id    : " << device_name.unique_id;
    // Add combination of user-friendly and unique name to the output list.
    device_names->emplace_back(device_name);
  }

  return true;
}

}  // namespace

bool CoreAudioUtility::IsSupported() {
  static bool g_is_supported = IsSupportedInternal();
  return g_is_supported;
}

int CoreAudioUtility::NumberOfActiveDevices(EDataFlow data_flow) {
  // Generate a collection of active audio endpoint devices for the specified
  // data-flow direction.
  ComPtr<IMMDeviceCollection> collection = CreateCollectionInternal(data_flow);
  if (!collection.Get()) {
    return 0;
  }

  // Retrieve the number of active audio devices for the specified direction.
  UINT number_of_active_devices = 0;
  collection->GetCount(&number_of_active_devices);
  std::string str;
  if (data_flow == eCapture) {
    str = "Number of capture devices: ";
  } else if (data_flow == eRender) {
    str = "Number of render devices: ";
  } else if (data_flow == eAll) {
    str = "Total number of devices: ";
  }
  RTC_DLOG(INFO) << str << number_of_active_devices;
  return static_cast<int>(number_of_active_devices);
}

ComPtr<IMMDeviceEnumerator> CoreAudioUtility::CreateDeviceEnumerator() {
  return CreateDeviceEnumeratorInternal(true);
}

ComPtr<IMMDevice> CoreAudioUtility::CreateDevice(const std::string& device_id,
                                                 EDataFlow data_flow,
                                                 ERole role) {
  return CreateDeviceInternal(device_id, data_flow, role);
}

AudioDeviceName CoreAudioUtility::GetDeviceName(IMMDevice* device) {
  AudioDeviceName device_name(GetDeviceFriendlyNameInternal(device),
                              GetDeviceIdInternal(device));
  RTC_DLOG(INFO) << "friendly name: " << device_name.device_name;
  RTC_DLOG(INFO) << "unique id    : " << device_name.unique_id;
  return device_name;
}

std::string CoreAudioUtility::GetFriendlyName(const std::string& device_id,
                                              EDataFlow data_flow,
                                              ERole role) {
  ComPtr<IMMDevice> audio_device = CreateDevice(device_id, data_flow, role);
  if (!audio_device.Get())
    return std::string();

  AudioDeviceName device_name = GetDeviceName(audio_device.Get());
  return device_name.device_name;
}

EDataFlow CoreAudioUtility::GetDataFlow(IMMDevice* device) {
  ComPtr<IMMEndpoint> endpoint;
  _com_error error = device->QueryInterface(endpoint.GetAddressOf());
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IMMDevice::QueryInterface failed: "
                      << ErrorToString(error);
    return eAll;
  }

  EDataFlow data_flow;
  error = endpoint->GetDataFlow(&data_flow);
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IMMEndpoint::GetDataFlow failed: "
                      << ErrorToString(error);
    return eAll;
  }
  return data_flow;
}

bool CoreAudioUtility::GetInputDeviceNames(AudioDeviceNames* device_names) {
  return GetDeviceNamesInternal(eCapture, device_names);
}

bool CoreAudioUtility::GetOutputDeviceNames(AudioDeviceNames* device_names) {
  return GetDeviceNamesInternal(eRender, device_names);
}

ComPtr<IAudioClient> CoreAudioUtility::CreateClient(
    const std::string& device_id,
    EDataFlow data_flow,
    ERole role) {
  ComPtr<IMMDevice> device(CreateDevice(device_id, data_flow, role));
  return CreateClientInternal(device.Get());
}

HRESULT CoreAudioUtility::GetSharedModeMixFormat(IAudioClient* client,
                                                 WAVEFORMATPCMEX* format) {
  ScopedCoMem<WAVEFORMATPCMEX> format_pcmex;
  _com_error error =
      client->GetMixFormat(reinterpret_cast<WAVEFORMATEX**>(&format_pcmex));
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioClient::GetMixFormat failed: "
                      << ErrorToString(error);
    return error.Error();
  }

  size_t bytes = sizeof(WAVEFORMATEX) + format_pcmex->Format.cbSize;
  RTC_DCHECK_EQ(bytes, sizeof(WAVEFORMATPCMEX))
      << "Format tag: 0x" << std::hex << format_pcmex->Format.wFormatTag;
  memcpy(format, format_pcmex, bytes);
  RTC_DLOG(INFO) << WaveFormatToString(format);

  return error.Error();
}

bool CoreAudioUtility::IsFormatSupported(IAudioClient* client,
                                         AUDCLNT_SHAREMODE share_mode,
                                         const WAVEFORMATPCMEX* format) {
  ScopedCoMem<WAVEFORMATEXTENSIBLE> closest_match;
  // This method provides a way for a client to determine, before calling
  // IAudioClient::Initialize, whether the audio engine supports a particular
  // stream format or not. In shared mode, the audio engine always supports
  // the mix format (see CoreAudioUtility::GetSharedModeMixFormat).
  // TODO(henrika): verify support for exclusive mode as well.
  _com_error error = client->IsFormatSupported(
      share_mode, reinterpret_cast<const WAVEFORMATEX*>(format),
      reinterpret_cast<WAVEFORMATEX**>(&closest_match));
  if ((error.Error() == S_OK) && (closest_match == nullptr)) {
    RTC_DLOG(INFO)
        << "The audio endpoint device supports the specified stream format";
  } else if ((error.Error() == S_FALSE) && (closest_match != nullptr)) {
    // Call succeeded with a closest match to the specified format. This log can
    // only be triggered for shared mode.
    RTC_LOG(LS_WARNING)
        << "Exact format is not supported, but a closest match exists";
    RTC_LOG(INFO) << WaveFormatToString(closest_match);
  } else if ((error.Error() == AUDCLNT_E_UNSUPPORTED_FORMAT) &&
             (closest_match == nullptr)) {
    // The audio engine does not support the caller-specified format or any
    // similar format.
    RTC_DLOG(INFO) << "The audio endpoint device does not support the "
                      "specified stream format";
  } else {
    RTC_LOG(LS_ERROR) << "IAudioClient::IsFormatSupported failed: "
                      << ErrorToString(error);
  }

  return (error.Error() == S_OK);
}

HRESULT CoreAudioUtility::GetDevicePeriod(IAudioClient* client,
                                          AUDCLNT_SHAREMODE share_mode,
                                          REFERENCE_TIME* device_period) {
  // The |default_period| parameter specifies the default scheduling period
  // for a shared-mode stream. The |minimum_period| parameter specifies the
  // minimum scheduling period for an exclusive-mode stream.
  // The time is expressed in 100-nanosecond units.
  REFERENCE_TIME default_period = 0;
  REFERENCE_TIME minimum_period = 0;
  _com_error error = client->GetDevicePeriod(&default_period, &minimum_period);
  if (error.Error() != S_OK) {
    RTC_LOG(LS_ERROR) << "IAudioClient::GetDevicePeriod failed: "
                      << ErrorToString(error);
    return error.Error();
  }

  *device_period = (share_mode == AUDCLNT_SHAREMODE_SHARED) ? default_period
                                                            : minimum_period;
  RTC_LOG(INFO) << "device_period: "
                << ReferenceTimeToTimeDelta(*device_period).ToMilliseconds()
                << " [ms]";
  return error.Error();
}

HRESULT CoreAudioUtility::GetPreferredAudioParameters(
    const std::string& device_id,
    bool is_output_device,
    AudioParameters* params) {
  _com_error error(S_OK);
  return error.Error();
}

}  // namespace win
}  // namespace webrtc
