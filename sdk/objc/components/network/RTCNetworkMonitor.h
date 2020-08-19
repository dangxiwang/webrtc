/*
 *  Copyright 2020 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

namespace webrtc {
class NetworkMonitorObserver;
}

/** Listens for NWPathMonitor updates and forwards the results to a C++
 *  observer.
 */
@interface RTCNetworkMonitor : NSObject

- (instancetype)init NS_UNAVAILABLE;

/** |observer| is a raw pointer and should be kept alive
 *  for this object's lifetime.
 */
- (instancetype)initWithObserver:(webrtc::NetworkMonitorObserver *)observer
    NS_DESIGNATED_INITIALIZER;

@end

NS_ASSUME_NONNULL_END
