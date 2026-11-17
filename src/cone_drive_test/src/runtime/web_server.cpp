#include "cone_drive_test/runtime/web_server.hpp"
#include "cone_drive_test/core/json_writer.hpp"
#include "cone_drive_test/core/mini_json.hpp"
#include "cone_drive_test/core/types.hpp"
#include <httplib.h>

namespace cone_drive_test {

namespace {
std::string reply_json(const Reply & r) {
  JsonWriter w;
  w.begin_object().key("ok").value(r.first).key("reason").value(r.second).end_object();
  return w.str();
}
void send(httplib::Response & res, const Reply & r) { res.set_content(reply_json(r), "application/json"); }
}  // namespace

WebServer::WebServer(std::string root, std::string bind, int port, const ParamRegistry & params, WebCallbacks cb, double timeout)
  : root_(std::move(root)), bind_(std::move(bind)), port_(port), params_(params), cb_(std::move(cb)),
    http_(std::make_unique<httplib::Server>()), timeout_(timeout) {
  streams_["bev"];
  streams_["camera"];
  routes();
}

WebServer::~WebServer() { stop(); }

void WebServer::start() {
  if (running_) return;
  http_->set_mount_point("/", root_);
  port_ = port_ > 0 ? (http_->bind_to_port(bind_, port_) ? port_ : -1) : http_->bind_to_any_port(bind_);
  if (port_ <= 0) throw std::runtime_error("web server: cannot bind " + bind_);
  running_ = true;
  thread_ = std::thread([this] { http_->listen_after_bind(); });
}

void WebServer::stop() {
  if (!running_.exchange(false)) return;
  cv_.notify_all();
  http_->stop();
  if (thread_.joinable()) thread_.join();
}

void WebServer::publish_telemetry(std::string json) {
  { std::lock_guard<std::mutex> l(mutex_); telemetry_ = std::move(json); ++telemetry_seq_; }
  cv_.notify_all();
}

void WebServer::publish_jpeg(const std::string & stream, std::vector<std::uint8_t> jpeg) {
  {
    std::lock_guard<std::mutex> l(mutex_);
    auto & s = streams_[stream];
    s.jpeg = std::move(jpeg);
    ++s.seq;
  }
  cv_.notify_all();
}

int WebServer::client_count(double now) const {
  std::lock_guard<std::mutex> l(mutex_);
  int n = 0;
  for (const auto & [id, c] : clients_) n += now - c.last <= timeout_;
  return n;
}

bool WebServer::preview_wanted(double now) const {
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto & [id, c] : clients_) if (now - c.last <= timeout_ && c.preview) return true;
  return false;
}

bool WebServer::stream_wanted(const std::string & stream, double now) const {
  std::lock_guard<std::mutex> l(mutex_);
  const auto it = streams_.find(stream);
  if (it == streams_.end() || it->second.viewers <= 0) return false;
  for (const auto & [id, c] : clients_)
    if (now - c.last <= timeout_ && c.preview && (stream == "bev" ? c.bev : c.camera)) return true;
  return false;
}

void WebServer::routes() {
  auto & s = *http_;
  auto body = [](const httplib::Request & req) { return parse_json(req.body.empty() ? std::string("{}") : req.body); };
  auto client_of = [](const JsonValue & j) { const auto * c = j.get("client"); return c && c->kind == JsonValue::String ? c->str : std::string("anonymous"); };

  s.Get("/api/params", [this](const httplib::Request &, httplib::Response & res) { res.set_content(params_.to_json(), "application/json"); });
  s.Post("/api/start", [this](const httplib::Request &, httplib::Response & res) { send(res, cb_.start ? cb_.start() : Reply{false, "unavailable"}); });
  s.Post("/api/stop", [this](const httplib::Request &, httplib::Response & res) { if (cb_.stop) cb_.stop(); send(res, {true, ""}); });
  s.Post("/api/refit", [this](const httplib::Request &, httplib::Response & res) { send(res, cb_.refit ? cb_.refit() : Reply{false, "unavailable"}); });
  s.Post("/api/params/save", [this](const httplib::Request &, httplib::Response & res) {
    send(res, cb_.params_save ? cb_.params_save() : Reply{false, "unavailable"});
  });
  s.Post("/api/mode", [this, body](const httplib::Request & req, httplib::Response & res) {
    const auto j = body(req);
    const auto * m = j ? j->get("mode") : nullptr;
    if (!m || m->kind != JsonValue::String) return send(res, {false, "body must be {\"mode\": \"drive\"|\"detect\"}"});
    send(res, cb_.mode ? cb_.mode(m->str) : Reply{false, "unavailable"});
  });
  s.Post("/api/params", [this, body](const httplib::Request & req, httplib::Response & res) {
    const auto j = body(req);
    const auto * ch = j ? j->get("changes") : nullptr;
    if (!ch || ch->kind != JsonValue::Object) return send(res, {false, "body must be {\"changes\": {name: number}}"});
    std::map<std::string, double> changes;
    for (const auto & [k, v] : ch->obj) {
      if (v->kind != JsonValue::Number) return send(res, {false, k + ": not a number"});
      changes[k] = v->num;
    }
    send(res, cb_.params ? cb_.params(changes) : Reply{false, "unavailable"});
  });
  s.Post("/api/heartbeat", [this, body, client_of](const httplib::Request & req, httplib::Response & res) {
    const auto j = body(req);
    const std::string id = j ? client_of(*j) : "anonymous";
    { std::lock_guard<std::mutex> l(mutex_); clients_[id].last = now_sec(); }
    if (cb_.heartbeat) cb_.heartbeat(id);
    send(res, {true, ""});
  });
  s.Post("/api/view", [this, body, client_of](const httplib::Request & req, httplib::Response & res) {
    const auto j = body(req);
    if (!j) return send(res, {false, "bad json"});
    std::lock_guard<std::mutex> l(mutex_);
    auto & c = clients_[client_of(*j)];
    c.last = now_sec();
    if (const auto * v = j->get("preview"); v && v->kind == JsonValue::Bool) c.preview = v->b;
    if (const auto * v = j->get("bev"); v && v->kind == JsonValue::Bool) c.bev = v->b;
    if (const auto * v = j->get("camera"); v && v->kind == JsonValue::Bool) c.camera = v->b;
    res.set_content(reply_json({true, ""}), "application/json");
  });
  s.Get("/api/events", [this](const httplib::Request &, httplib::Response & res) {
    res.set_header("Cache-Control", "no-cache");
    auto seen = std::make_shared<std::uint64_t>(0);
    res.set_chunked_content_provider("text/event-stream", [this, seen](size_t, httplib::DataSink & sink) {
      std::string msg;
      {
        std::unique_lock<std::mutex> l(mutex_);
        cv_.wait_for(l, std::chrono::seconds(1), [&] { return !running_ || telemetry_seq_ != *seen; });
        if (!running_) return false;
        if (telemetry_seq_ != *seen && !telemetry_.empty()) { *seen = telemetry_seq_; msg = "data: " + telemetry_ + "\n\n"; }
        else msg = ": keepalive\n\n";
      }
      return sink.write(msg.data(), msg.size());
    });
  });
  for (const std::string name : {"bev", "camera"}) {
    s.Get("/api/stream/" + name, [this, name](const httplib::Request &, httplib::Response & res) {
      { std::lock_guard<std::mutex> l(mutex_); ++streams_[name].viewers; }
      auto seen = std::make_shared<std::uint64_t>(0);
      res.set_chunked_content_provider("multipart/x-mixed-replace; boundary=frame",
        [this, name, seen](size_t, httplib::DataSink & sink) {
          std::vector<std::uint8_t> jpeg;
          {
            std::unique_lock<std::mutex> l(mutex_);
            auto & st = streams_[name];
            cv_.wait_for(l, std::chrono::seconds(1), [&] { return !running_ || st.seq != *seen; });
            if (!running_) return false;
            if (st.seq == *seen || st.jpeg.empty()) return true;
            *seen = st.seq;
            jpeg = st.jpeg;
          }
          const std::string head = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(jpeg.size()) + "\r\n\r\n";
          return sink.write(head.data(), head.size()) && sink.write(reinterpret_cast<const char *>(jpeg.data()), jpeg.size()) &&
                 sink.write("\r\n", 2);
        },
        [this, name](bool) { std::lock_guard<std::mutex> l(mutex_); --streams_[name].viewers; });
    });
  }
}

}  // namespace cone_drive_test
