// BEV 캔버스 좌표 검사: 오버레이(P)와 배경 BEV 영상(0~3 m × ±0.6 m, 120×300)이 같은 축척·위치여야 한다.
//   node src/cone_drive_test/test/web/bev_geometry.test.js   (playwright 필요)
const { chromium } = require("playwright");
const http = require("http");
const fs = require("fs");
const path = require("path");
const root = path.resolve(__dirname, "../../web");
const types = { ".html": "text/html", ".js": "text/javascript", ".css": "text/css", ".json": "application/json", ".jpg": "image/jpeg" };
const server = http.createServer((req, res) => {
  const f = path.join(root, decodeURIComponent(req.url.split("?")[0]));
  fs.readFile(f, (e, d) => { if (e) { res.writeHead(404); res.end(); } else { res.writeHead(200, { "Content-Type": types[path.extname(f)] || "application/octet-stream" }); res.end(d); } });
});
server.listen(0, async () => {
  const b = await chromium.launch();
  const p = await b.newPage();
  await p.goto(`http://localhost:${server.address().port}/index.html?demo&still&bev=1`);
  await p.waitForTimeout(500);
  const g = await p.evaluate(() => {
    const cv = document.getElementById("bev");
    return { w: cv.width, h: cv.height, cssW: cv.getBoundingClientRect().width, cssH: cv.getBoundingClientRect().height,
             far: P(3.0, 0.6), near: P(0.0, -0.6), rect: bevImageRect(), scale };
  });
  await b.close();
  server.close();
  const fails = [];
  const near = (a, e) => Math.abs(a - e) < 1e-6;
  // 오버레이 축척으로 본 영상 영역 = 배경을 그리는 영역
  if (!(near(g.rect.x, g.far[0]) && near(g.rect.y, g.far[1]) && near(g.rect.w, g.near[0] - g.far[0]) && near(g.rect.h, g.near[1] - g.far[1])))
    fails.push(`image rect ${JSON.stringify(g.rect)} != overlay box ${JSON.stringify([g.far, g.near])}`);
  // 영상 1 px = 1 cm 이 가로·세로 같은 축척 (120:300)
  if (!near(g.rect.w / 120, g.rect.h / 300)) fails.push(`image aspect stretched: ${g.rect.w}x${g.rect.h}`);
  // 캔버스가 표시 범위를 다 담고 CSS로 늘어나지 않음
  if (!near(g.cssW, g.w) || !near(g.cssH, g.h)) fails.push(`canvas css ${g.cssW}x${g.cssH} != ${g.w}x${g.h}`);
  if (g.near[1] > g.h) fails.push(`x=0 at ${g.near[1]}px is below canvas height ${g.h}`);
  console.log(fails.length ? "FAIL\n" + fails.join("\n") : "PASS", JSON.stringify(g));
  process.exit(fails.length ? 1 : 0);
});
