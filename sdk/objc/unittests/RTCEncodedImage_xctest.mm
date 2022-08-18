/*
 *  Copyright 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "api/peerconnection/RTCEncodedImage+Private.h"

#import <XCTest/XCTest.h>

#if defined(WEBRTC_MAC) && !defined(WEBRTC_IOS)
#include "sdk/objc/unittests/xctest_to_gtest.h"
#endif

@interface RTCEncodedImageTests : XCTestCase
@end

@implementation RTCEncodedImageTests

- (void)testInitializedWithNativeEncodedImage {
  const auto encoded_data = webrtc::EncodedImageBuffer::Create();
  webrtc::EncodedImage encoded_image;
  encoded_image.SetEncodedData(encoded_data);

  RTC_OBJC_TYPE(RTCEncodedImage) *encodedImage =
      [[RTC_OBJC_TYPE(RTCEncodedImage) alloc] initWithNativeEncodedImage:encoded_image];

  XCTAssertEqual([encodedImage nativeEncodedImage].GetEncodedData(), encoded_data);
}

- (void)testInitWithNSData {
  NSData *bufferData = [NSData data];
  RTC_OBJC_TYPE(RTCEncodedImage) *encodedImage = [[RTC_OBJC_TYPE(RTCEncodedImage) alloc] init];
  encodedImage.buffer = bufferData;

  webrtc::EncodedImage result_encoded_image = [encodedImage nativeEncodedImage];
  XCTAssertTrue(result_encoded_image.GetEncodedData() != nullptr);
  XCTAssertEqual(result_encoded_image.GetEncodedData()->data(), bufferData.bytes);
}

- (void)testRetainsNativeEncodedImage {
  RTC_OBJC_TYPE(RTCEncodedImage) * encodedImage;
  {
    const auto encoded_data = webrtc::EncodedImageBuffer::Create();
    webrtc::EncodedImage encoded_image;
    encoded_image.SetEncodedData(encoded_data);
    encodedImage =
        [[RTC_OBJC_TYPE(RTCEncodedImage) alloc] initWithNativeEncodedImage:encoded_image];
  }
  webrtc::EncodedImage result_encoded_image = [encodedImage nativeEncodedImage];
  XCTAssertTrue(result_encoded_image.GetEncodedData() != nullptr);
  XCTAssertTrue(result_encoded_image.GetEncodedData()->data() != nullptr);
}

@end

#if defined(WEBRTC_MAC) && !defined(WEBRTC_IOS)

namespace webrtc {

class RTCEncodedImageTest : public XCTestToGTest<RTCEncodedImageTests> {
 public:
  RTCEncodedImageTest() = default;
  ~RTCEncodedImageTest() override = default;
};

INVOKE_XCTEST(RTCEncodedImageTest, InitializedWithNativeEncodedImage)

INVOKE_XCTEST(RTCEncodedImageTest, InitWithNSData)

INVOKE_XCTEST(RTCEncodedImageTest, RetainsNativeEncodedImage)

}  // namespace webrtc

#endif  // defined(WEBRTC_MAC) && !defined(WEBRTC_IOS)
