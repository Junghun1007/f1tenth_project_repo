// 내장 웹 서버 (cpp-httplib): 정적 페이지, SSE 텔레메트리, MJPEG(BEV·카메라), 명령 POST, 클라이언트별 heartbeat·보기 설정
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "cone_drive_test/core/params.hpp"

namespace httplib { class Server; }

namespace cone_drive_test {

using Reply = std::pair<bool, std::string>;
struct WebCallbacks {
  std::function<Reply()> start, refit, params_save;
  std::function<void()> stop;
  std::function<Reply(const std::string &)> mode, detector;
  std::function<Reply(const std::map<std::string, double> &)> params;
  std::function<void(const std::string &)> heartbeat;
  std::function<Reply(const std::string & label)> record_start;   // 수동 주행 녹화 시작 (label: 상황 이름)
  std::function<Reply()> record_stop;
};

class WebServer {
public:
  WebServer(std::string web_root, std::string bind, int port, const ParamRegistry & params, WebCallbacks callbacks,
            double client_timeout_sec = 1.0);
  ~WebServer();
  void start();
  void stop();
  int port() const { return port_; }

  void publish_telemetry(std::string json);
  void publish_jpeg(const std::string & stream, std::vector<std::uint8_t> jpeg);   // "bev" | "camera"

  int client_count(double now) const;
  bool preview_wanted(double now) const;                              // 살아 있는 클라이언트 중 하나라도 프리뷰 켬
  bool stream_wanted(const std::string & stream, double now) const;   // 프리뷰+해당 보기 켬, MJPEG 연결 열림
  void set_client_timeout(double s) { std::lock_guard<std::mutex> l(mutex_); timeout_ = s; }

private:
  struct Client { double last{0}; bool preview{true}, bev{false}, camera{false}; };
  struct Stream { std::vector<std::uint8_t> jpeg; std::uint64_t seq{0}; int viewers{0}; };
  void routes();
  std::string root_, bind_;
  int port_;
  const ParamRegistry & params_;
  WebCallbacks cb_;
  std::unique_ptr<httplib::Server> http_;
  std::thread thread_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<bool> running_{false};
  double timeout_;
  std::map<std::string, Client> clients_;
  std::string telemetry_;
  std::uint64_t telemetry_seq_{0};
  std::map<std::string, Stream> streams_;
};

}  // namespace cone_drive_test
