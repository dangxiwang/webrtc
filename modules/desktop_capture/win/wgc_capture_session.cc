/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/desktop_capture/win/wgc_capture_session.h"

#include <Windows.Graphics.Capture.Interop.h>
#include <Windows.Graphics.DirectX.Direct3d11.interop.h>
#include <wrl.h>
#include <memory>
#include <utility>
#include <vector>

#include "modules/desktop_capture/win/d3d_device.h"
#include "rtc_base/logging.h"
#include "rtc_base/win/get_activation_factory.h"

using Microsoft::WRL::ComPtr;
namespace WGC = ABI::Windows::Graphics::Capture;

// We must use a BGRA pixel format that has 4 bytes per pixel, as required by
// the DesktopFrame interface.
const auto kPixelFormat = ABI::Windows::Graphics::DirectX::DirectXPixelFormat::
    DirectXPixelFormat_B8G8R8A8UIntNormalized;

// We only want 1 buffer in our frame pool to reduce latency. If we had more,
// they would sit in the pool for longer and be stale by the time we are asked
// for a new frame.
const int kNumBuffers = 1;

namespace webrtc {

WgcCaptureSession::WgcCaptureSession(ComPtr<ID3D11Device> d3d11_device,
                                     HWND window)
    : d3d11_device_(std::move(d3d11_device)), window_(window) {}
WgcCaptureSession::~WgcCaptureSession() = default;

HRESULT WgcCaptureSession::StartCapture() {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  RTC_DCHECK(!is_capture_started_);
  RTC_DCHECK(d3d11_device_);
  RTC_DCHECK(window_);

  if (window_closed_ || !window_) {
    RTC_LOG(LS_ERROR) << "The target window has been closed.";
    return E_ABORT;
  }

  if (!rtc_base::win::ResolveCoreWinRTDelayload())
    return E_FAIL;

  ComPtr<IGraphicsCaptureItemInterop> interop;
  HRESULT hr = rtc_base::win::GetActivationFactory<
      IGraphicsCaptureItemInterop,
      RuntimeClass_Windows_Graphics_Capture_GraphicsCaptureItem>(&interop);
  if (FAILED(hr))
    return hr;

  hr = interop->CreateForWindow(window_, IID_PPV_ARGS(&item_));
  if (FAILED(hr)) {
    RTC_LOG(LS_ERROR) << "Failed to create graphics capture item: " << hr;
    return hr;
  }

  // Listen for the Closed event, to detect if the user closes the window we are
  // capturing. If they do, we should abort the capture.
  auto closed_handler =
      Microsoft::WRL::Callback<ABI::Windows::Foundation::ITypedEventHandler<
          WGC::GraphicsCaptureItem*, IInspectable*>>(
          this, &WgcCaptureSession::OnItemClosed);
  EventRegistrationToken item_closed_token;
  item_->add_Closed(closed_handler.Get(), &item_closed_token);

  // This DXGI device is unused, but it is necessary to get an IDirect3DDevice.
  ComPtr<IDXGIDevice> dxgi_device;
  hr = d3d11_device_->QueryInterface(IID_PPV_ARGS(&dxgi_device));
  if (FAILED(hr))
    return hr;

  ComPtr<IInspectable> inspectableSurface;
  hr = CreateDirect3D11DeviceFromDXGIDevice(dxgi_device.Get(),
                                            &inspectableSurface);
  if (FAILED(hr))
    return hr;

  hr = inspectableSurface->QueryInterface(IID_PPV_ARGS(&direct3d_device_));
  if (FAILED(hr))
    return hr;

  ComPtr<WGC::IDirect3D11CaptureFramePoolStatics> frame_pool_statics;
  hr = rtc_base::win::GetActivationFactory<
      WGC::IDirect3D11CaptureFramePoolStatics,
      RuntimeClass_Windows_Graphics_Capture_Direct3D11CaptureFramePool>(
      &frame_pool_statics);
  if (FAILED(hr))
    return hr;

  // Cast to FramePoolStatics2 so we can use CreateFreeThreaded and avoid the
  // need to have a DispatcherQueue. We don't listen for the FrameArrived event,
  // so there's no difference.
  ComPtr<WGC::IDirect3D11CaptureFramePoolStatics2> frame_pool_statics2;
  hr = frame_pool_statics->QueryInterface(IID_PPV_ARGS(&frame_pool_statics2));
  if (FAILED(hr))
    return hr;

  ABI::Windows::Graphics::SizeInt32 item_size;
  hr = item_.Get()->get_Size(&item_size);
  if (FAILED(hr))
    return hr;

  previous_size_ = item_size;
  hr = frame_pool_statics2->CreateFreeThreaded(direct3d_device_.Get(),
                                               kPixelFormat, kNumBuffers,
                                               item_size, &frame_pool_);
  if (FAILED(hr))
    return hr;

  hr = frame_pool_->CreateCaptureSession(item_.Get(), &session_);
  if (FAILED(hr))
    return hr;

  hr = session_->StartCapture();
  if (FAILED(hr)) {
    RTC_LOG(LS_ERROR) << "Failed to start CaptureSession: " << hr;
    return hr;
  }

  is_capture_started_ = true;
  return hr;
}

HRESULT WgcCaptureSession::GetMostRecentFrame(
    std::unique_ptr<DesktopFrame>* output_frame) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);

  if (window_closed_) {
    RTC_LOG(LS_ERROR) << "The target window has been closed.";
    return E_ABORT;
  }

  RTC_DCHECK(is_capture_started_);

  ComPtr<WGC::IDirect3D11CaptureFrame> capture_frame;
  HRESULT hr = frame_pool_->TryGetNextFrame(&capture_frame);
  if (FAILED(hr)) {
    RTC_LOG(LS_ERROR) << "TryGetNextFrame failed: " << hr;
    return hr;
  }

  if (!capture_frame) {
    RTC_LOG(LS_INFO) << "TryGetNextFrame was empty: " << capture_frame;
    return hr;
  }

  // We need to get this CaptureFrame as an ID3D11Texture2D so that we can get
  // the raw image data in the format required by the DesktopFrame interface.
  ComPtr<ABI::Windows::Graphics::DirectX::Direct3D11::IDirect3DSurface>
      d3d_surface;
  hr = capture_frame->get_Surface(&d3d_surface);
  if (FAILED(hr))
    return hr;

  ComPtr<Windows::Graphics::DirectX::Direct3D11::IDirect3DDxgiInterfaceAccess>
      direct3DDxgiInterfaceAccess;
  hr = d3d_surface->QueryInterface(IID_PPV_ARGS(&direct3DDxgiInterfaceAccess));
  if (FAILED(hr))
    return hr;

  ComPtr<ID3D11Texture2D> texture_2D;
  hr = direct3DDxgiInterfaceAccess->GetInterface(IID_PPV_ARGS(&texture_2D));
  if (FAILED(hr))
    return hr;

  if (!mapped_texture_) {
    hr = CreateMappedTexture(texture_2D, texture_2D.Width, texutre_2D.Height);
    if (FAILED(hr))
      return hr;
  }

  // We need to copy |texture_2D| into |mapped_texture_| as the latter has the
  // D3D11_CPU_ACCESS_READ flag set, which lets us access the image data.
  // Otherwise it would only be readable by the GPU.
  ComPtr<ID3D11DeviceContext> d3d_context;
  d3d11_device_->GetImmediateContext(&d3d_context);
  d3d_context->CopyResource(mapped_texture_.Get(), texture_2D.Get());

  D3D11_MAPPED_SUBRESOURCE map_info;
  hr = d3d_context->Map(mapped_texture_.Get(), /*subresource_index=*/0,
                        D3D11_MAP_READ, /*D3D11_MAP_FLAG_DO_NOT_WAIT=*/0,
                        &map_info);
  if (FAILED(hr))
    return hr;

  ABI::Windows::Graphics::SizeInt32 new_size;
  hr = capture_frame->get_ContentSize(&new_size);
  if (FAILED(hr))
    return hr;

  // If the size has changed since the last capture, we must be sure to choose
  // the smaller of the two sizes. Otherwise we might overrun our buffer, or
  // read stale data from the last frame.
  int previous_area = previous_size_.Width * previous_size_.Height;
  int new_area = new_size.Width * new_size.Height;
  auto smaller_size = previous_area < new_area ? previous_size_ : new_size;

  // Make a copy of the data pointed to by |map_info.pData| so we are free to
  // unmap our texture.
  uint8_t* data = static_cast<uint8_t*>(map_info.pData);
  int data_size = smaller_size.Height * map_info.RowPitch;
  std::vector<uint8_t> image_data(data, data + data_size);
  DesktopSize size(smaller_size.Width, smaller_size.Height);
  *output_frame = std::make_unique<DesktopFrameWinWgc>(
      size, static_cast<int>(map_info.RowPitch), std::move(image_data));

  d3d_context->Unmap(mapped_texture_.Get(), 0);

  // If the size changed, we must resize the texture and frame pool to fit the
  // new size.
  if (previous_area != new_area) {
    hr = CreateMappedTexture(texture_2D, new_size.Width, new_size.Height);
    if (FAILED(hr))
      return hr;

    hr = frame_pool_->Recreate(direct3d_device_.Get(), kPixelFormat,
                               kNumBuffers, new_size);
    if (FAILED(hr))
      return hr;
  }

  previous_size_ = new_size;
  return hr;
}

HRESULT WgcCaptureSession::CreateMappedTexture(
    ComPtr<ID3D11Texture2D> src_texture,
    UINT width,
    UINT height) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);

  D3D11_TEXTURE2D_DESC src_desc;
  src_texture->GetDesc(&src_desc);
  D3D11_TEXTURE2D_DESC map_desc;
  map_desc.Width = width;
  map_desc.Height = height;
  map_desc.MipLevels = src_desc.MipLevels;
  map_desc.ArraySize = src_desc.ArraySize;
  map_desc.Format = src_desc.Format;
  map_desc.SampleDesc = src_desc.SampleDesc;
  map_desc.Usage = D3D11_USAGE_STAGING;
  map_desc.BindFlags = 0;
  map_desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
  map_desc.MiscFlags = 0;
  return d3d11_device_->CreateTexture2D(&map_desc, nullptr, &mapped_texture_);
}

HRESULT WgcCaptureSession::OnItemClosed(WGC::IGraphicsCaptureItem* sender,
                                        IInspectable* event_args) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);

  RTC_LOG(LS_INFO) << "Capture target has been closed.";
  window_closed_ = true;
  is_capture_started_ = false;
  mapped_texture_ = nullptr;
  session_ = nullptr;
  frame_pool_ = nullptr;
  direct3d_device_ = nullptr;
  item_ = nullptr;
  d3d11_device_ = nullptr;

  return S_OK;
}

}  // namespace webrtc
