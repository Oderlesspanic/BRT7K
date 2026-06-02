const ros = new ROSLIB.Ros({
  url: `ws://${window.location.hostname}:9090`
});

const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");
const mapStatus = document.getElementById("mapStatus");

let mapBitmap = null;
let mapWidth = 0;
let mapHeight = 0;
let mapResolution = 0;
let mapOrigin = { x: 0, y: 0, yaw: 0 };
let drawBounds = { x: 0, y: 0, scale: 1, width: 0, height: 0 };
let robotPose = null;
const transforms = new Map();
const ROBOT_MESH_BOUNDS = {
  minX: -0.3484,
  minY: -0.2356,
  maxX: 0.2277,
  maxY: 0.2344
};
const ROBOT_MESH_YAW_OFFSET = Math.PI;
const MAP_VIEW_YAW = Math.PI / 2;
const robotImage = new Image();
robotImage.src = "assets/robot_top.png?v=20260602-5";
robotImage.addEventListener("load", resizeAndDraw);

const mapTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/map",
  messageType: "nav_msgs/OccupancyGrid"
});

const tfTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/tf",
  messageType: "tf2_msgs/TFMessage",
  throttle_rate: 100
});

const tfStaticTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/tf_static",
  messageType: "tf2_msgs/TFMessage"
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
  mapResolution = msg.info.resolution;
  mapOrigin = {
    x: msg.info.origin.position.x,
    y: msg.info.origin.position.y,
    yaw: yawFromQuaternion(msg.info.origin.orientation)
  };
  mapBitmap = buildMapImage(msg);
  mapStatus.textContent = `${mapWidth} x ${mapHeight}`;
  resizeAndDraw();
});

tfTopic.subscribe(updateTransforms);
tfStaticTopic.subscribe(updateTransforms);

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

  const viewWidth = mapHeight;
  const viewHeight = mapWidth;
  const scale = Math.min(window.innerWidth / viewWidth, window.innerHeight / viewHeight);
  const drawWidth = viewWidth * scale;
  const drawHeight = viewHeight * scale;
  const x = (window.innerWidth - drawWidth) * 0.5;
  const y = (window.innerHeight - drawHeight) * 0.5;

  drawBounds = { x, y, scale, width: drawWidth, height: drawHeight };
  ctx.save();
  ctx.translate(x + drawWidth * 0.5, y + drawHeight * 0.5);
  ctx.rotate(MAP_VIEW_YAW);
  ctx.drawImage(
    mapBitmap,
    -mapWidth * scale * 0.5,
    -mapHeight * scale * 0.5,
    mapWidth * scale,
    mapHeight * scale
  );
  ctx.restore();
  drawRobot();
}

function updateTransforms(msg) {
  for (const transform of msg.transforms || []) {
    transforms.set(transformKey(transform.header.frame_id, transform.child_frame_id), transform);
  }

  robotPose = resolveRobotPose();
  resizeAndDraw();
}

function resolveRobotPose() {
  const direct = getTransform("map", "base_link") || getTransform("map", "base_footprint");
  if (direct) return poseFromTransform(direct);

  const mapToOdom = getTransform("map", "odom");
  const odomToBase = getTransform("odom", "base_link") || getTransform("odom", "base_footprint");
  if (!mapToOdom || !odomToBase) return robotPose;

  return composePoses(poseFromTransform(mapToOdom), poseFromTransform(odomToBase));
}

function getTransform(parent, child) {
  return transforms.get(transformKey(parent, child)) || null;
}

function transformKey(parent, child) {
  return `${normalizeFrame(parent)}>${normalizeFrame(child)}`;
}

function normalizeFrame(frame) {
  return String(frame || "").replace(/^\/+/, "");
}

function poseFromTransform(transform) {
  return {
    x: transform.transform.translation.x,
    y: transform.transform.translation.y,
    yaw: yawFromQuaternion(transform.transform.rotation)
  };
}

function composePoses(a, b) {
  const cos = Math.cos(a.yaw);
  const sin = Math.sin(a.yaw);

  return {
    x: a.x + cos * b.x - sin * b.y,
    y: a.y + sin * b.x + cos * b.y,
    yaw: normalizeAngle(a.yaw + b.yaw)
  };
}

function worldToCanvas(x, y) {
  if (!mapResolution || mapWidth === 0 || mapHeight === 0) return null;

  const dx = x - mapOrigin.x;
  const dy = y - mapOrigin.y;
  const cos = Math.cos(-mapOrigin.yaw);
  const sin = Math.sin(-mapOrigin.yaw);
  const mapX = (cos * dx - sin * dy) / mapResolution;
  const mapY = (sin * dx + cos * dy) / mapResolution;

  const localX = mapX * drawBounds.scale;
  const localY = (mapHeight - mapY) * drawBounds.scale;
  const unrotatedCenterX = mapWidth * drawBounds.scale * 0.5;
  const unrotatedCenterY = mapHeight * drawBounds.scale * 0.5;
  const rotatedX = drawBounds.width * 0.5 - (localY - unrotatedCenterY);
  const rotatedY = drawBounds.height * 0.5 + (localX - unrotatedCenterX);

  return {
    x: drawBounds.x + rotatedX,
    y: drawBounds.y + rotatedY
  };
}

function drawRobot() {
  if (!robotPose || !mapBitmap || mapResolution === 0) return;

  const center = worldToCanvas(robotPose.x, robotPose.y);
  if (!center) return;

  const metersToCanvas = drawBounds.scale / mapResolution;
  const imageX = ROBOT_MESH_BOUNDS.minX * metersToCanvas;
  const imageY = -ROBOT_MESH_BOUNDS.maxY * metersToCanvas;
  const imageWidth = (ROBOT_MESH_BOUNDS.maxX - ROBOT_MESH_BOUNDS.minX) * metersToCanvas;
  const imageHeight = (ROBOT_MESH_BOUNDS.maxY - ROBOT_MESH_BOUNDS.minY) * metersToCanvas;
  const yaw = -robotPose.yaw + mapOrigin.yaw + ROBOT_MESH_YAW_OFFSET + MAP_VIEW_YAW;

  ctx.save();
  ctx.translate(center.x, center.y);
  ctx.rotate(yaw);

  if (robotImage.complete && robotImage.naturalWidth > 0) {
    ctx.drawImage(robotImage, imageX, imageY, imageWidth, imageHeight);
  }

  ctx.restore();
}

function yawFromQuaternion(q) {
  if (!q) return 0;
  const siny = 2 * (q.w * q.z + q.x * q.y);
  const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
  return Math.atan2(siny, cosy);
}

function normalizeAngle(angle) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

resizeAndDraw();
