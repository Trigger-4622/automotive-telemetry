/*
 * Host mock of the Arduino-ESP32 WebServer: handlers are recorded, and the
 * harness calls them directly with request() — no sockets.
 */
#pragma once

#include <Arduino.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "FS.h"

enum HTTPMethod { HTTP_ANY, HTTP_GET, HTTP_POST, HTTP_PUT, HTTP_DELETE };
enum HTTPUploadStatus { UPLOAD_FILE_START, UPLOAD_FILE_WRITE, UPLOAD_FILE_END,
                        UPLOAD_FILE_ABORTED };

struct HTTPUpload {
    HTTPUploadStatus status = UPLOAD_FILE_START;
    String  filename;
    size_t  totalSize = 0, currentSize = 0;
    uint8_t buf[1436] = {};
};

class WebServer {
public:
    using Handler = std::function<void()>;
    explicit WebServer(int port = 80) { (void)port; }
    void on(const char *uri, HTTPMethod m, Handler h) { routes_.push_back({uri, m, h, nullptr}); }
    void on(const char *uri, HTTPMethod m, Handler h, Handler up) { routes_.push_back({uri, m, h, up}); }
    void onNotFound(Handler h) { notFound_ = h; }
    void begin() {}
    void handleClient() {}

    String arg(const char *name) {
        auto it = args_.find(name);
        return it == args_.end() ? String() : String(it->second);
    }
    String uri() { return String(uri_); }
    HTTPUpload &upload() { return upload_; }

    void sendHeader(const String &k, const String &v, bool = false) {
        headers[k.str()] = v.str();
    }
    void send(int code, const char *type, const String &body) {
        this->code = code; contentType = type; this->body = body.str();
    }
    void send(int code, const char *type, const char *body) { send(code, type, String(body)); }
    void send_P(int code, const char *type, const char *data, size_t len) {
        this->code = code; contentType = type; body.assign(data, len);
    }
    size_t streamFile(File &f, const String &type) {
        std::string s; char b[256]; size_t n;
        while ((n = f.readBytes(b, sizeof b)) > 0) s.append(b, n);
        code = 200; contentType = type.str(); body = s;
        return s.size();
    }

    /** Harness: run the handler for one request, as the network would. */
    bool request(HTTPMethod m, const std::string &uri, const std::string &plain = "") {
        code = 0; body.clear(); headers.clear(); contentType.clear();
        uri_ = uri; args_.clear(); args_["plain"] = plain;
        for (auto &r : routes_)
            if (r.uri == uri && (r.m == m || r.m == HTTP_ANY)) { r.h(); return true; }
        if (notFound_) notFound_();
        return false;
    }

    /** Harness: a multipart upload of @p data as @p filename - the upload
     *  handler in the chunks the real server hands over, then the route's own
     *  handler (which the real server skips after an abort). */
    bool upload(const std::string &uri, const std::string &filename,
                const std::string &data, bool abort = false) {
        code = 0; body.clear(); headers.clear(); contentType.clear();
        uri_ = uri; args_.clear();
        for (auto &r : routes_) {
            if (r.uri != uri || r.m != HTTP_POST || !r.up) continue;
            upload_ = HTTPUpload();
            upload_.filename = String(filename.c_str());
            upload_.status = UPLOAD_FILE_START;
            r.up();
            for (size_t at = 0; at < data.size(); at += sizeof upload_.buf) {
                upload_.currentSize = std::min(sizeof upload_.buf, data.size() - at);
                memcpy(upload_.buf, data.data() + at, upload_.currentSize);
                upload_.totalSize += upload_.currentSize;
                upload_.status = UPLOAD_FILE_WRITE;
                r.up();
            }
            upload_.status = abort ? UPLOAD_FILE_ABORTED : UPLOAD_FILE_END;
            r.up();
            if (!abort) r.h();
            return true;
        }
        return false;
    }

    int code = 0;
    std::string contentType, body;
    std::map<std::string, std::string> headers;

private:
    struct Route { std::string uri; HTTPMethod m; Handler h, up; };
    std::vector<Route> routes_;
    Handler notFound_;
    std::string uri_;
    std::map<std::string, std::string> args_;
    HTTPUpload upload_;
};
