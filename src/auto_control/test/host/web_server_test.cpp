#include "check.hpp"
#include "auto_control/drive/runtime/web_server.hpp"
#include "auto_control/drive/core/types.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <thread>

using namespace auto_control::drive;

struct Fixture {
  ParamRegistry params;
  std::atomic<int> heartbeats{0}, starts{0};
  std::atomic<bool> force_ir{false};
  std::string last_client;
  std::unique_ptr<WebServer> server;
  Fixture() {
    params.add({"drive.maximum_speed_mps", "driving", "속도", "최대 속도", "m/s", 0.0, 3.0, 1.8, true});
    WebCallbacks cb;
    cb.start = [this] { ++starts; return std::make_pair(false, std::string("detect mode")); };
    cb.stop = [] {};
    cb.force_ir = [this](bool enabled) { force_ir = enabled; return Reply{true, ""}; };
    cb.refit = [] { return std::make_pair(true, std::string()); };
    cb.mode = [](const std::string & m) { return std::make_pair(m == "drive" || m == "detect", std::string("bad mode")); };
    cb.params = [this](const std::map<std::string, double> & c) { return params.apply(c); };
    cb.params_save = [] { return std::make_pair(true, std::string("/tmp/x.yaml")); };
    cb.heartbeat = [this](const std::string & id) { ++heartbeats; last_client = id; };
    server = std::make_unique<WebServer>(CDT_WEB, "127.0.0.1", 0, params, cb, 1.0);
    server->start();
  }
  ~Fixture() { server->stop(); }
  httplib::Client client() { httplib::Client c("127.0.0.1", server->port()); c.set_read_timeout(2, 0); return c; }
};

TEST(serves_index_and_params) {
  Fixture f;
  auto c = f.client();
  auto r = c.Get("/");
  CHECK(r && r->status == 200);
  CHECK(r->body.find("auto_control") != std::string::npos);
  r = c.Get("/api/params");
  CHECK(r && r->status == 200);
  const auto j = nlohmann::json::parse(r->body);
  CHECK(j["params"][0]["name"] == "drive.maximum_speed_mps");
}

TEST(params_post_rejects_bad_value) {
  Fixture f;
  auto c = f.client();
  auto r = c.Post("/api/params", R"({"changes":{"drive.maximum_speed_mps":9.5}})", "application/json");
  CHECK(r && r->status == 200);
  auto j = nlohmann::json::parse(r->body);
  CHECK(j["ok"] == false);
  CHECK(j["reason"].get<std::string>().find("drive.maximum_speed_mps") != std::string::npos);
  r = c.Post("/api/params", R"({"changes":{"drive.maximum_speed_mps":1.25}})", "application/json");
  j = nlohmann::json::parse(r->body);
  CHECK(j["ok"] == true);
  CHECK_NEAR(f.params.get("drive.maximum_speed_mps"), 1.25, 1e-12);
  r = c.Post("/api/params", "not json", "application/json");
  CHECK(r && nlohmann::json::parse(r->body)["ok"] == false);
}

TEST(start_returns_reason) {
  Fixture f;
  auto c = f.client();
  const auto r = c.Post("/api/start", "{}", "application/json");
  const auto j = nlohmann::json::parse(r->body);
  CHECK(j["ok"] == false && j["reason"] == "detect mode");
  CHECK(f.starts == 1);
}

TEST(tunnel_override_accepts_only_boolean_requests) {
  Fixture f;
  auto c = f.client();
  const auto post = [&c](const char * body) {
    const auto r = c.Post("/api/tunnel/force-ir", body, "application/json");
    CHECK(r && r->status == 200);
    return nlohmann::json::parse(r->body);
  };
  CHECK(post(R"({"enabled":true})")["ok"] == true);
  CHECK(f.force_ir.load());
  CHECK(post(R"({"enabled":"false"})")["ok"] == false);
  CHECK(f.force_ir.load());
  CHECK(post(R"({"enabled":false})")["ok"] == true);
  CHECK(!f.force_ir.load());
}

TEST(write_parameter_yaml_replaces_only_when_asked) {   // 카메라 초기화가 새 컨테이너에 넘기는 YAML
  struct Directory {
    std::filesystem::path path;
    ~Directory() { std::error_code error; std::filesystem::remove_all(path, error); }
  };
  auto tmp = (std::filesystem::temp_directory_path() / "auto-control-yaml-XXXXXX").string();
  CHECK(::mkdtemp(tmp.data()) != nullptr);
  Directory dir{tmp};
  const auto file = dir.path / "reset.yaml";
  const auto read = [&file] { std::ifstream in(file); return std::string(std::istreambuf_iterator<char>(in), {}); };
  write_parameter_yaml(file, "a: 1\n", false);
  CHECK(read() == "a: 1\n");
  bool kept = false;
  try { write_parameter_yaml(file, "a: 2\n", false); } catch (const std::filesystem::filesystem_error &) { kept = true; }
  CHECK(kept && read() == "a: 1\n");
  write_parameter_yaml(file, "a: 3\n", true);
  CHECK(read() == "a: 3\n");
  std::size_t entries = 0;
  for (const auto & entry : std::filesystem::directory_iterator(dir.path)) { (void)entry; ++entries; }
  CHECK(entries == 1);   // 임시 파일이 남지 않음
}

TEST(reset_and_save_endpoints_dispatch_and_presets_are_gone) {
  ParamRegistry params;
  WebCallbacks cb;
  cb.camera_reset = [] { return Reply{false, "vehicle moving"}; };
  cb.params_save = [] { return Reply{true, "/tmp/auto_drive_saved.yaml"}; };
  WebServer server(CDT_WEB, "127.0.0.1", 0, params, cb);
  server.start();
  httplib::Client client("127.0.0.1", server.port());
  const auto post = [&client](const char * path, const char * body) {
    const auto response = client.Post(path, body, "application/json");
    CHECK(response && response->status == 200);
    return nlohmann::json::parse(response->body);
  };
  CHECK(post("/api/camera/reset", "{}")["reason"] == "vehicle moving");
  CHECK(post("/api/params/save", "{}")["reason"] == "/tmp/auto_drive_saved.yaml");
  const auto list = client.Get("/api/presets");
  CHECK(!list || list->status == 404);
  const auto load = client.Post("/api/presets/load", R"({"name":"x.yaml"})", "application/json");
  CHECK(!load || load->status == 404);
}

TEST(heartbeat_tracks_clients_and_preview_flag) {
  Fixture f;
  auto c = f.client();
  c.Post("/api/heartbeat", R"({"client":"abc"})", "application/json");
  CHECK(f.heartbeats == 1 && f.last_client == "abc");
  const double now = now_sec();
  CHECK(f.server->client_count(now) == 1);
  CHECK(f.server->preview_wanted(now));        // 기본은 프리뷰 켜짐
  c.Post("/api/view", R"({"client":"abc","preview":false})", "application/json");
  CHECK(!f.server->preview_wanted(now_sec()));
  CHECK(f.server->client_count(now + 1.5) == 0);   // heartbeat 만료
}

TEST(sse_delivers_published_telemetry) {
  Fixture f;
  std::string got;
  std::thread reader([&] {
    auto c = f.client();
    c.Get("/api/events", [&](const char * data, size_t n) {
      got.append(data, n);
      return got.find("\"hello\"") == std::string::npos;   // 원하는 이벤트를 받으면 중단
    });
  });
  for (int k = 0; k < 40 && got.find("hello") == std::string::npos; ++k) {
    f.server->publish_telemetry(R"({"state":"hello"})");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  reader.join();
  CHECK(got.find("data: {\"state\":\"hello\"}") != std::string::npos);
}

TEST(bev_stream_wanted_only_with_viewer) {
  Fixture f;
  auto c = f.client();
  c.Post("/api/heartbeat", R"({"client":"abc"})", "application/json");
  CHECK(!f.server->stream_wanted("bev", now_sec()));
  c.Post("/api/view", R"({"client":"abc","bev":true})", "application/json");
  CHECK(!f.server->stream_wanted("bev", now_sec()));   // 아직 MJPEG 연결 없음
  std::atomic<bool> done{false};
  std::string got;
  std::thread viewer([&] {
    auto v = f.client();
    v.Get("/api/stream/bev", [&](const char * d, size_t n) { got.append(d, n); return !done; });
  });
  bool wanted = false;
  for (int k = 0; k < 40 && !wanted; ++k) { std::this_thread::sleep_for(std::chrono::milliseconds(25)); wanted = f.server->stream_wanted("bev", now_sec()); }
  CHECK(wanted);
  f.server->publish_jpeg("bev", std::vector<std::uint8_t>{0xff, 0xd8, 0xff, 0xd9});
  for (int k = 0; k < 40 && got.find("image/jpeg") == std::string::npos; ++k) std::this_thread::sleep_for(std::chrono::milliseconds(25));
  done = true;
  f.server->publish_jpeg("bev", std::vector<std::uint8_t>{0xff, 0xd8, 0xff, 0xd9});
  viewer.join();
  CHECK(got.find("image/jpeg") != std::string::npos);
}
