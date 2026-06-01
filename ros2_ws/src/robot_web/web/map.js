const ros = new ROSLIB.Ros({
  url: `ws://${window.location.hostname}:9090`
});

const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");
const mapStatus = document.getElementById("mapStatus");

let mapBitmap = null;
let mapWidth = 0;
let mapHeight = 0;

const mapTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/map",
  messageType: "nav_msgs/OccupancyGrid"
});

ros.on("connection", () => {
  mapStatus.textContent = "verbunden, warte auf Karte";
});

ros.on("close", () => {
  mapStatus.textContent = "ROS getrennt";
});

ros.on("error", () => {
  mapStatus.textContent = "ROS Fehler";
});

mapTopic.subscribe((msg) => {
  mapWidth = msg.info.width;
  mapHeight = msg.info.height;
  mapBitmap = buildMapImage(msg);
  mapStatus.textContent = `${mapWidth} x ${mapHeight}`;
  resizeAndDraw();
});

window.addEventListener("resize", resizeAndDraw);

function buildMapImage(msg) {
  const offscreen = document.createElement("canvas");
  offscreen.width = msg.info.width;
  offscreen.height = msg.info.height;

  const offscreenCtx = offscreen.getContext("2d");
  const imageData = offscreenCtx.createImageData(msg.info.width, msg.info.height);

  for (let y = 0; y < msg.info.height; y++) {
    for (let x = 0; x < msg.info.width; x++) {
      const mapIndex = x + (msg.info.height - y - 1) * msg.info.width;
      const imageIndex = (x + y * msg.info.width) * 4;
      const value = msg.data[mapIndex];
      let color = 0;

      if (value === -1) color = 120;
      else if (value === 0) color = 245;

      imageData.data[imageIndex + 0] = color;
      imageData.data[imageIndex + 1] = color;
      imageData.data[imageIndex + 2] = color;
      imageData.data[imageIndex + 3] = 255;
    }
  }

  offscreenCtx.putImageData(imageData, 0, 0);
  return offscreen;
}

function resizeAndDraw() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(window.innerWidth * dpr);
  canvas.height = Math.floor(window.innerHeight * dpr);
  canvas.style.width = `${window.innerWidth}px`;
  canvas.style.height = `${window.innerHeight}px`;

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.fillStyle = "#111";
  ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);

  if (!mapBitmap || mapWidth === 0 || mapHeight === 0) return;

  const scale = Math.min(window.innerWidth / mapWidth, window.innerHeight / mapHeight);
  const drawWidth = mapWidth * scale;
  const drawHeight = mapHeight * scale;
  const x = (window.innerWidth - drawWidth) * 0.5;
  const y = (window.innerHeight - drawHeight) * 0.5;

  ctx.drawImage(mapBitmap, x, y, drawWidth, drawHeight);
}

resizeAndDraw();
