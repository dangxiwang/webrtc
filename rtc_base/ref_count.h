/*
 *  Copyright 2011 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef RTC_BASE_REF_COUNT_H_
#define RTC_BASE_REF_COUNT_H_

namespace rtc {

// Refcounted objects should implement the following informal interface:
//
// void AddRef() const ;
// void Release() const;
//
// You may access members of a reference-counted object, including the AddRef()
// and Release() methods, only if you already own a reference to it, or if
// you're borrowing someone else's reference. (A newly created object is a
// special case: the reference count is zero on construction, and the code that
// creates the object should immediately call AddRef(), bringing the reference
// count from zero to one, e.g., by constructing an rtc::scoped_refptr).
//
// AddRef() creates a new reference to the object.
//
// Release() releases a reference to the object; the caller now has one less
// reference than before the call. If the number of references dropped to zero
// because of this the object must destroy itself.
//
// The caller of Release() must treat it in the same way as a delete operation:
// The caller mustn't access the object. The object might still be alive, due to
// references held by other users of the object, but the object can go away at
// any time, e.g., as the result of another thread calling Release().
//
// Calling AddRef() and Release() manually is discouraged. It's recommended to
// use rtc::scoped_refptr to manage all pointers to reference counted objects.
// Note that rtc::scoped_refptr depends on compile-time duck-typing; formally
// implementing the below RefCountInterface is not required.

// Interfaces where refcounting is part of the public api should
// inherit this abstract interface. The implementation of these
// methods is usually provided by the RefCountedObject template class,
// applied as a leaf in the inheritance tree.
class RefCountInterface {
 public:
  virtual void AddRef() const = 0;
  virtual void Release() const = 0;

 protected:
  // Non-public destructor, because Release() has exclusive responsibility for
  // destroying the object.
  virtual ~RefCountInterface() = default;
};

}  // namespace rtc

#endif  // RTC_BASE_REF_COUNT_H_
