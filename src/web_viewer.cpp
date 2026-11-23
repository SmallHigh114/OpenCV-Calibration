#include "web_viewer.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace qd {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

static std::string url_decode(const std::string& str) {
    std::string out;
    out.reserve(str.size());
    for (size_t i = 0; i < str.size(); ++i) {
        if (str[i] == '%' && i + 2 < str.size()) {
            int val = 0;
            std::istringstream iss(str.substr(i + 1, 2));
            iss >> std::hex >> val;
            out += static_cast<char>(val);
            i += 2;
        } else if (str[i] == '+') {
            out += ' ';
        } else {
            out += str[i];
        }
    }
    return out;
}

static std::string query_param(const std::string& path,
                               const std::string& param) {
    auto qpos = path.find('?');
    if (qpos == std::string::npos) return "";
    std::string query = path.substr(qpos + 1);
    size_t pos = 0;
    while (pos < query.size()) {
        auto eq  = query.find('=', pos);
        auto amp = query.find('&', pos);
        if (amp == std::string::npos) amp = query.size();
        if (eq != std::string::npos && eq < amp) {
            if (query.substr(pos, eq - pos) == param)
                return url_decode(query.substr(eq + 1, amp - eq - 1));
        }
        pos = amp + 1;
    }
    return "";
}

// ---------------------------------------------------------------------------
// HTML page (embedded)
// ---------------------------------------------------------------------------

static const char* HTML_PAGE = R"html(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenCV WebViewer</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1e1e2e;color:#cdd6f4;font-family:system-ui,-apple-system,sans-serif;
     display:flex;flex-direction:column;height:100vh;overflow:hidden}
header{background:#181825;padding:14px 24px;display:flex;align-items:center;
       justify-content:space-between;border-bottom:1px solid #313244}
header h1{font-size:18px;font-weight:600;letter-spacing:.5px}
header .dot{width:10px;height:10px;border-radius:50%;background:#a6e3a1;
            display:inline-block;margin-right:8px}
header .dot.off{background:#f38ba8}
main{flex:1;display:flex;flex-wrap:wrap;justify-content:center;align-items:center;
     gap:20px;padding:20px;overflow:hidden;min-height:0}
.card{background:#313244;border-radius:10px;overflow:hidden;
      box-shadow:0 4px 16px rgba(0,0,0,.35);min-width:320px;max-width:90vw}
.card .title{padding:10px 16px;background:#45475a;font-size:13px;font-weight:500;
             letter-spacing:.3px;display:flex;align-items:center;gap:8px}
.card .title .icon{opacity:.6}
.card img{display:block;max-width:100%;max-height:calc(100vh - 140px);object-fit:contain;background:#11111b}
.empty{color:#6c7086;font-size:15px;padding:40px;text-align:center}
footer{background:#181825;padding:10px 24px;font-size:12px;color:#6c7086;
       border-top:1px solid #313244;display:flex;justify-content:space-between}
.toast{position:fixed;bottom:60px;left:50%;transform:translateX(-50%);
       background:#b4befe;color:#1e1e2e;padding:6px 18px;border-radius:20px;
       font-size:13px;font-weight:600;opacity:0;transition:opacity .2s;pointer-events:none}
.toast.show{opacity:1}
.help{position:fixed;top:60px;right:16px;background:rgba(49,50,68,.92);
      padding:14px 18px;border-radius:10px;font-size:12px;line-height:2;
      backdrop-filter:blur(8px);border:1px solid #45475a}
.help b{color:#b4befe}
.help kbd{background:#45475a;padding:1px 6px;border-radius:4px;font-family:monospace}
</style>
</head>
<body tabindex="0">
<header>
  <h1><span class="dot" id="dot"></span>OpenCV WebViewer</h1>
  <span id="hdr-info" style="font-size:12px;color:#6c7086"></span>
</header>
<main id="container">
  <div class="empty" id="placeholder">等待视频流连接 ...</div>
</main>
<footer>
  <span id="status">初始化中...</span>
  <span>点击页面后可使用键盘控制</span>
</footer>
<div class="toast" id="toast"></div>
<div class="help">
  <b>快捷键</b><br>
  <kbd>S</kbd> 采集  <kbd>C</kbd> 标定<br>
  <kbd>A</kbd> 自动采集  <kbd>R</kbd> 重置<br>
  <kbd>ESC</kbd> 退出
</div>
<script>
const known = new Map();
let toastTimer = null;

function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 600);
}

document.body.addEventListener('keydown', e => {
  e.preventDefault();
  let code;
  if (e.key === 'Escape') code = 27;
  else if (e.key === 'Enter') code = 13;
  else if (e.key === 'Backspace') code = 8;
  else if (e.key.length === 1) code = e.key.charCodeAt(0);
  else return;
  showToast('按键: ' + (code === 27 ? 'ESC' : e.key) + ' (' + code + ')');
  fetch('/key', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({key: code})
  }).catch(() => {});
});

async function poll() {
  try {
    const r = await fetch('/api/windows');
    const wins = await r.json();
    const container = document.getElementById('container');
    const ph = document.getElementById('placeholder');
    const current = new Set(wins);
    wins.forEach(name => {
      if (!known.has(name)) {
        if (ph) ph.style.display = 'none';
        const card = document.createElement('div');
        card.className = 'card';
        card.id = 'w-' + name;
        const ts = Date.now();
        card.innerHTML =
          '<div class="title"><span class="icon">&#9654;</span>' + name + '</div>' +
          '<img src="/stream?window=' + encodeURIComponent(name) + '&t=' + ts + '" alt="' + name + '">';
        container.appendChild(card);
        known.set(name, card);
      }
    });
    known.forEach((el, name) => {
      if (!current.has(name)) { el.remove(); known.delete(name); }
    });
    document.getElementById('dot').className = 'dot';
    document.getElementById('status').textContent =
      '已连接 | ' + wins.length + ' 个窗口';
    document.getElementById('hdr-info').textContent =
      wins.length ? wins.join(', ') : '';
  } catch {
    document.getElementById('dot').className = 'dot off';
    document.getElementById('status').textContent = '连接断开 ...';
  }
}

setInterval(poll, 2000);
poll();
document.body.focus();
</script>
</body>
</html>)html";

// ---------------------------------------------------------------------------
// WebViewer implementation
// ---------------------------------------------------------------------------

WebViewer::WebViewer(int port, int jpeg_quality)
    : port_(port), jpeg_quality_(jpeg_quality) {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "[WebViewer] socket() 失败: " << strerror(errno)
                  << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(static_cast<uint16_t>(port_));

    if (bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) <
        0) {
        std::cerr << "[WebViewer] bind() 端口 " << port_ << " 失败: "
                  << strerror(errno) << std::endl;
        close(server_fd_);
        server_fd_ = -1;
        return;
    }

    if (listen(server_fd_, 32) < 0) {
        std::cerr << "[WebViewer] listen() 失败: " << strerror(errno)
                  << std::endl;
        close(server_fd_);
        server_fd_ = -1;
        return;
    }

    running_       = true;
    server_thread_ = std::thread(&WebViewer::server_loop, this);

    std::cout << "[WebViewer] 服务已启动: http://localhost:" << port_
              << std::endl;
}

WebViewer::~WebViewer() {
    running_ = false;
    win_cv_.notify_all();
    key_cv_.notify_all();

    if (server_fd_ >= 0) {
        shutdown(server_fd_, SHUT_RDWR);
        close(server_fd_);
        server_fd_ = -1;
    }

    if (server_thread_.joinable()) server_thread_.join();

    {
        std::lock_guard<std::mutex> lock(fds_mtx_);
        for (int fd : active_fds_) shutdown(fd, SHUT_RDWR);
    }

    while (active_handlers_ > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// ---------------------------------------------------------------------------
// public API
// ---------------------------------------------------------------------------

void WebViewer::namedWindow(const std::string& winname) {
    std::lock_guard<std::mutex> lock(win_mtx_);
    windows_[winname];
}

void WebViewer::imshow(const std::string& winname, const cv::Mat& img) {
    std::lock_guard<std::mutex> lock(win_mtx_);
    auto& wd = windows_[winname];
    img.copyTo(wd.frame);
    ++wd.seq;
    win_cv_.notify_all();
}

int WebViewer::waitKey(int delay_ms) {
    std::unique_lock<std::mutex> lock(key_mtx_);
    auto pred = [this] { return !keys_.empty() || !running_; };

    if (delay_ms <= 0)
        key_cv_.wait(lock, pred);
    else
        key_cv_.wait_for(lock, std::chrono::milliseconds(delay_ms), pred);

    if (!keys_.empty()) {
        int k = keys_.front();
        keys_.pop();
        return k;
    }
    return -1;
}

void WebViewer::destroyWindow(const std::string& winname) {
    std::lock_guard<std::mutex> lock(win_mtx_);
    windows_.erase(winname);
}

void WebViewer::destroyAllWindows() {
    std::lock_guard<std::mutex> lock(win_mtx_);
    windows_.clear();
}

// ---------------------------------------------------------------------------
// server
// ---------------------------------------------------------------------------

void WebViewer::server_loop() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t   addr_len  = sizeof(client_addr);
        int         client_fd = accept(
            server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client_fd < 0) {
            if (running_)
                std::cerr << "[WebViewer] accept() 失败: " << strerror(errno)
                          << std::endl;
            continue;
        }

        // 5s 读写超时，防止线程永远阻塞
        struct timeval tv { 5, 0 };
        setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        std::thread([this, client_fd] {
            {
                std::lock_guard<std::mutex> lock(fds_mtx_);
                active_fds_.insert(client_fd);
            }
            ++active_handlers_;

            handle_client(client_fd);

            {
                std::lock_guard<std::mutex> lock(fds_mtx_);
                active_fds_.erase(client_fd);
            }
            close(client_fd);
            --active_handlers_;
        }).detach();
    }
}

// ---------------------------------------------------------------------------
// request handling
// ---------------------------------------------------------------------------

void WebViewer::handle_client(int fd) {
    char buf[8192];
    int  n = recv(fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return;
    buf[n] = '\0';
    std::string request(buf, static_cast<size_t>(n));

    auto line_end = request.find("\r\n");
    if (line_end == std::string::npos) return;

    std::istringstream iss(request.substr(0, line_end));
    std::string        method, path, version;
    iss >> method >> path >> version;

    // 取 path 的基础部分（不含 query string）
    std::string base_path = path;
    auto        qpos      = path.find('?');
    if (qpos != std::string::npos) base_path = path.substr(0, qpos);

    if (method == "GET") {
        if (base_path == "/") {
            send_html_page(fd);
        } else if (base_path == "/stream") {
            send_mjpeg_stream(fd, query_param(path, "window"));
        } else if (base_path == "/api/windows") {
            send_window_list(fd);
        } else {
            send_response(fd, 404, "text/plain", "Not Found");
        }
    } else if (method == "POST" && base_path == "/key") {
        auto body_start = request.find("\r\n\r\n");
        if (body_start != std::string::npos) {
            std::string body     = request.substr(body_start + 4);
            auto        key_pos  = body.find("\"key\"");
            if (key_pos != std::string::npos) {
                auto colon = body.find(':', key_pos);
                if (colon != std::string::npos) {
                    try {
                        int key_code = std::stoi(body.substr(colon + 1));
                        {
                            std::lock_guard<std::mutex> lock(key_mtx_);
                            keys_.push(key_code);
                        }
                        key_cv_.notify_one();
                    } catch (...) {}
                }
            }
        }
        send_response(fd, 200, "text/plain", "OK");
    } else if (method == "OPTIONS") {
        // CORS preflight
        std::string headers =
            "HTTP/1.1 204 No Content\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
            "Access-Control-Allow-Headers: Content-Type\r\n"
            "Content-Length: 0\r\n\r\n";
        send(fd, headers.data(), headers.size(), MSG_NOSIGNAL);
    } else {
        send_response(fd, 405, "text/plain", "Method Not Allowed");
    }
}

// ---------------------------------------------------------------------------
// response helpers
// ---------------------------------------------------------------------------

void WebViewer::send_response(int fd, int code,
                              const std::string& content_type,
                              const std::string& body) {
    const char* status_text = "OK";
    switch (code) {
        case 200: status_text = "200 OK"; break;
        case 204: status_text = "204 No Content"; break;
        case 404: status_text = "404 Not Found"; break;
        case 405: status_text = "405 Method Not Allowed"; break;
        default:  status_text = "200 OK"; break;
    }
    std::ostringstream oss;
    oss << "HTTP/1.1 " << status_text << "\r\n"
        << "Content-Type: " << content_type << "\r\n"
        << "Content-Length: " << body.size() << "\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        << "Connection: close\r\n"
        << "\r\n"
        << body;
    std::string resp = oss.str();
    send(fd, resp.data(), resp.size(), MSG_NOSIGNAL);
}

void WebViewer::send_html_page(int fd) {
    send_response(fd, 200, "text/html; charset=utf-8", HTML_PAGE);
}

void WebViewer::send_window_list(int fd) {
    std::lock_guard<std::mutex> lock(win_mtx_);
    std::ostringstream          json;
    json << "[";
    bool first = true;
    for (auto& [name, _] : windows_) {
        if (!first) json << ",";
        json << "\"" << name << "\"";
        first = false;
    }
    json << "]";
    send_response(fd, 200, "application/json", json.str());
}

// ---------------------------------------------------------------------------
// MJPEG streaming
// ---------------------------------------------------------------------------

void WebViewer::send_mjpeg_stream(int fd, const std::string& window) {
    // MJPEG 流不设置读写超时，保持长连接
    struct timeval tv_long { 0, 0 };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv_long, sizeof(tv_long));

    std::string header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
    if (send(fd, header.data(), header.size(), MSG_NOSIGNAL) < 0) return;

    uint64_t          last_seq = 0;
    std::vector<uchar> jpeg_buf;
    std::vector<int>   params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

    while (running_) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(win_mtx_);
            win_cv_.wait_for(lock, std::chrono::milliseconds(100), [&] {
                auto it = windows_.find(window);
                return (it != windows_.end() && it->second.seq > last_seq) ||
                       !running_;
            });
            if (!running_) break;

            auto it = windows_.find(window);
            if (it == windows_.end() || it->second.seq == last_seq ||
                it->second.frame.empty())
                continue;

            frame    = it->second.frame.clone();
            last_seq = it->second.seq;
        }

        jpeg_buf.clear();
        if (!cv::imencode(".jpg", frame, jpeg_buf, params)) continue;

        std::ostringstream part;
        part << "--frame\r\n"
             << "Content-Type: image/jpeg\r\n"
             << "Content-Length: " << jpeg_buf.size() << "\r\n"
             << "\r\n";
        std::string hdr = part.str();

        if (send(fd, hdr.data(), hdr.size(), MSG_NOSIGNAL) < 0) break;
        if (send(fd, reinterpret_cast<const char*>(jpeg_buf.data()),
                 jpeg_buf.size(), MSG_NOSIGNAL) < 0)
            break;
        if (send(fd, "\r\n", 2, MSG_NOSIGNAL) < 0) break;
    }
}

} // namespace qd
