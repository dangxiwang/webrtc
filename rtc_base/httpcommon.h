/*
 *  Copyright 2004 The WebRTC Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef RTC_BASE_HTTPCOMMON_H_
#define RTC_BASE_HTTPCOMMON_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rtc_base/checks.h"
#include "rtc_base/stream.h"
#include "rtc_base/stringutils.h"

namespace rtc {

class CryptString;
class SocketAddress;

//////////////////////////////////////////////////////////////////////
// Constants
//////////////////////////////////////////////////////////////////////

enum HttpCode {
  HC_OK = 200,
  HC_INTERNAL_SERVER_ERROR = 500,
};

enum HttpVersion { HVER_1_0, HVER_1_1, HVER_UNKNOWN, HVER_LAST = HVER_UNKNOWN };

enum HttpError {
  HE_NONE,
  HE_PROTOCOL,             // Received non-valid HTTP data
  HE_DISCONNECTED,         // Connection closed unexpectedly
  HE_OVERFLOW,             // Received too much data for internal buffers
  HE_CONNECT_FAILED,       // The socket failed to connect.
  HE_SOCKET_ERROR,         // An error occurred on a connected socket
  HE_SHUTDOWN,             // Http object is being destroyed
  HE_OPERATION_CANCELLED,  // Connection aborted locally
  HE_AUTH,                 // Proxy Authentication Required
  HE_CERTIFICATE_EXPIRED,  // During SSL negotiation
  HE_STREAM,               // Problem reading or writing to the document
  HE_CACHE,                // Problem reading from cache
  HE_DEFAULT
};

enum HttpHeader {
  HH_AGE,
  HH_CACHE_CONTROL,
  HH_CONNECTION,
  HH_CONTENT_DISPOSITION,
  HH_CONTENT_LENGTH,
  HH_CONTENT_RANGE,
  HH_CONTENT_TYPE,
  HH_COOKIE,
  HH_DATE,
  HH_ETAG,
  HH_EXPIRES,
  HH_HOST,
  HH_IF_MODIFIED_SINCE,
  HH_IF_NONE_MATCH,
  HH_KEEP_ALIVE,
  HH_LAST_MODIFIED,
  HH_LOCATION,
  HH_PROXY_AUTHENTICATE,
  HH_PROXY_AUTHORIZATION,
  HH_PROXY_CONNECTION,
  HH_RANGE,
  HH_SET_COOKIE,
  HH_TE,
  HH_TRAILERS,
  HH_TRANSFER_ENCODING,
  HH_UPGRADE,
  HH_USER_AGENT,
  HH_WWW_AUTHENTICATE,
  HH_LAST = HH_WWW_AUTHENTICATE
};

const uint16_t HTTP_DEFAULT_PORT = 80;
const uint16_t HTTP_SECURE_PORT = 443;

//////////////////////////////////////////////////////////////////////
// Utility Functions
//////////////////////////////////////////////////////////////////////

const char* ToString(HttpVersion version);
bool FromString(HttpVersion& version, const std::string& str);

const char* ToString(HttpHeader header);
bool FromString(HttpHeader& header, const std::string& str);

bool HttpHeaderIsEndToEnd(HttpHeader header);
bool HttpHeaderIsCollapsible(HttpHeader header);

struct HttpData;
bool HttpShouldKeepAlive(const HttpData& data);

typedef std::pair<std::string, std::string> HttpAttribute;
typedef std::vector<HttpAttribute> HttpAttributeList;
void HttpParseAttributes(const char* data,
                         size_t len,
                         HttpAttributeList& attributes);
bool HttpHasAttribute(const HttpAttributeList& attributes,
                      const std::string& name,
                      std::string* value);
bool HttpHasNthAttribute(HttpAttributeList& attributes,
                         size_t index,
                         std::string* name,
                         std::string* value);

// Convert RFC1123 date (DoW, DD Mon YYYY HH:MM:SS TZ) to unix timestamp
bool HttpDateToSeconds(const std::string& date, time_t* seconds);

inline uint16_t HttpDefaultPort(bool secure) {
  return secure ? HTTP_SECURE_PORT : HTTP_DEFAULT_PORT;
}

// Returns the http server notation for a given address
std::string HttpAddress(const SocketAddress& address, bool secure);

// functional for insensitive std::string compare
struct iless {
  bool operator()(const std::string& lhs, const std::string& rhs) const {
    return (::_stricmp(lhs.c_str(), rhs.c_str()) < 0);
  }
};

// put quotes around a string and escape any quotes inside it
std::string quote(const std::string& str);

//////////////////////////////////////////////////////////////////////
// HttpData
//////////////////////////////////////////////////////////////////////

struct HttpData {
  typedef std::multimap<std::string, std::string, iless> HeaderMap;
  typedef HeaderMap::const_iterator const_iterator;
  typedef HeaderMap::iterator iterator;

  HttpVersion version;
  std::unique_ptr<StreamInterface> document;

  HttpData();

  enum HeaderCombine { HC_YES, HC_NO, HC_AUTO, HC_REPLACE, HC_NEW };
  void changeHeader(const std::string& name,
                    const std::string& value,
                    HeaderCombine combine);
  inline void addHeader(const std::string& name,
                        const std::string& value,
                        bool append = true) {
    changeHeader(name, value, append ? HC_AUTO : HC_NO);
  }
  inline void setHeader(const std::string& name,
                        const std::string& value,
                        bool overwrite = true) {
    changeHeader(name, value, overwrite ? HC_REPLACE : HC_NEW);
  }
  // Returns count of erased headers
  size_t clearHeader(const std::string& name);
  // Returns iterator to next header
  iterator clearHeader(iterator header);

  // keep in mind, this may not do what you want in the face of multiple headers
  bool hasHeader(const std::string& name, std::string* value) const;

  inline const_iterator begin() const { return headers_.begin(); }
  inline const_iterator end() const { return headers_.end(); }
  inline iterator begin() { return headers_.begin(); }
  inline iterator end() { return headers_.end(); }
  inline const_iterator begin(const std::string& name) const {
    return headers_.lower_bound(name);
  }
  inline const_iterator end(const std::string& name) const {
    return headers_.upper_bound(name);
  }
  inline iterator begin(const std::string& name) {
    return headers_.lower_bound(name);
  }
  inline iterator end(const std::string& name) {
    return headers_.upper_bound(name);
  }

  // Convenience methods using HttpHeader
  inline void changeHeader(HttpHeader header,
                           const std::string& value,
                           HeaderCombine combine) {
    changeHeader(ToString(header), value, combine);
  }
  inline void addHeader(HttpHeader header,
                        const std::string& value,
                        bool append = true) {
    addHeader(ToString(header), value, append);
  }
  inline void setHeader(HttpHeader header,
                        const std::string& value,
                        bool overwrite = true) {
    setHeader(ToString(header), value, overwrite);
  }
  inline void clearHeader(HttpHeader header) { clearHeader(ToString(header)); }
  inline bool hasHeader(HttpHeader header, std::string* value) const {
    return hasHeader(ToString(header), value);
  }
  inline const_iterator begin(HttpHeader header) const {
    return headers_.lower_bound(ToString(header));
  }
  inline const_iterator end(HttpHeader header) const {
    return headers_.upper_bound(ToString(header));
  }
  inline iterator begin(HttpHeader header) {
    return headers_.lower_bound(ToString(header));
  }
  inline iterator end(HttpHeader header) {
    return headers_.upper_bound(ToString(header));
  }

  void setContent(const std::string& content_type, StreamInterface* document);
  void setDocumentAndLength(StreamInterface* document);

  virtual size_t formatLeader(char* buffer, size_t size) const = 0;
  virtual HttpError parseLeader(const char* line, size_t len) = 0;

 protected:
  virtual ~HttpData();
  void clear(bool release_document);

 private:
  HeaderMap headers_;
};

struct HttpRequestData : public HttpData {
  std::string path;

  HttpRequestData() {}

  void clear(bool release_document);

  size_t formatLeader(char* buffer, size_t size) const override;
  HttpError parseLeader(const char* line, size_t len) override;
};

struct HttpResponseData : public HttpData {
  uint32_t scode;
  std::string message;

  HttpResponseData() : scode(HC_INTERNAL_SERVER_ERROR) {}
  void clear(bool release_document);

  // Convenience methods
  void set_success(uint32_t scode = HC_OK);
  void set_error(uint32_t scode);

  size_t formatLeader(char* buffer, size_t size) const override;
  HttpError parseLeader(const char* line, size_t len) override;
};

struct HttpTransaction {
  HttpRequestData request;
  HttpResponseData response;
};

//////////////////////////////////////////////////////////////////////
// Http Authentication
//////////////////////////////////////////////////////////////////////

struct HttpAuthContext {
  std::string auth_method;
  HttpAuthContext(const std::string& auth) : auth_method(auth) {}
  virtual ~HttpAuthContext() {}
};

enum HttpAuthResult { HAR_RESPONSE, HAR_IGNORE, HAR_CREDENTIALS, HAR_ERROR };

// 'context' is used by this function to record information between calls.
// Start by passing a null pointer, then pass the same pointer each additional
// call.  When the authentication attempt is finished, delete the context.
// TODO(bugs.webrtc.org/8905): Change "response" to "ZeroOnFreeBuffer".
HttpAuthResult HttpAuthenticate(const char* challenge,
                                size_t len,
                                const SocketAddress& server,
                                const std::string& method,
                                const std::string& uri,
                                const std::string& username,
                                const CryptString& password,
                                HttpAuthContext*& context,
                                std::string& response,
                                std::string& auth_method);

//////////////////////////////////////////////////////////////////////

}  // namespace rtc

#endif  // RTC_BASE_HTTPCOMMON_H_
