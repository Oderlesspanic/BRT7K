const ros = new ROSLIB.Ros({
  url: `ws://${window.location.hostname}:9090`
});

const rosStatus = document.getElementById("rosStatus");

ros.on("connection", () => {
  rosStatus.innerText = "verbunden";
  rosStatus.className = "status connected";
});

ros.on("error", () => {
  rosStatus.innerText = "Fehler";
  rosStatus.className = "status disconnected";
});

ros.on("close", () => {
  rosStatus.innerText = "getrennt";
  rosStatus.className = "status disconnected";
});

// ----------------------------------------------------
// Map: /map nav_msgs/OccupancyGrid
// ----------------------------------------------------

const mapTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/map",
  messageType: "nav_msgs/OccupancyGrid"
});

const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");

mapTopic.subscribe((msg) => {
  const width = msg.info.width;
  const height = msg.info.height;
  const data = msg.data;

  canvas.width = width;
  canvas.height = height;

  const imageData = ctx.createImageData(width, height);

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const mapIndex = x + (height - y - 1) * width;
      const imageIndex = (x + y * width) * 4;

      const value = data[mapIndex];

      let color;
      if (value === -1) color = 180;
      else if (value === 0) color = 255;
      else color = 0;

      imageData.data[imageIndex + 0] = color;
      imageData.data[imageIndex + 1] = color;
      imageData.data[imageIndex + 2] = color;
      imageData.data[imageIndex + 3] = 255;
    }
  }

  ctx.putImageData(imageData, 0, 0);
});

// ----------------------------------------------------
// Battery: /battery_state sensor_msgs/BatteryState
// voltage -> V
// current -> A
// ----------------------------------------------------

const batteryTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/battery_state",
  messageType: "sensor_msgs/BatteryState"
});

batteryTopic.subscribe((msg) => {
  document.getElementById("batteryVoltage").innerText =
    msg.voltage.toFixed(2) + " V";

  document.getElementById("currentDraw").innerText =
    msg.current.toFixed(2) + " A";
});

// ----------------------------------------------------
// Kontrollleuchten: /system_status std_msgs/String
//
// Erwarteter JSON-Inhalt:
// {
//   "lidar": "green",
//   "camera": "green",
//   "esp32_drive": "green",
//   "esp32_gripper": "yellow",
//   "imu": "green",
//   "magnetometer": "red"
// }
// ----------------------------------------------------

const systemStatusTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/system_status",
  messageType: "std_msgs/String"
});

systemStatusTopic.subscribe((msg) => {
  try {
    const status = JSON.parse(msg.data);

    setLamp("lidar", status.lidar);
    setLamp("camera", status.camera);
    setLamp("esp_drive", status.esp32_drive);
    setLamp("esp_gripper", status.esp32_gripper);
    setLamp("imu", status.imu);
    setLamp("magnetometer", status.magnetometer);

  } catch (e) {
    console.error("Fehler beim Parsen von /system_status:", e);
  }
});

function setLamp(name, color) {
  const lamp = document.getElementById("lamp_" + name);
  if (!lamp) return;

  lamp.className = "lamp";

  if (color === "green") {
    lamp.classList.add("green");
  } else if (color === "yellow") {
    lamp.classList.add("yellow");
  } else {
    lamp.classList.add("red");
  }
}

// ----------------------------------------------------
// Steuerung: /cmd_vel geometry_msgs/Twist
// ----------------------------------------------------

let speedPercent = 30;
let manualMode = true;
let mappingActive = false;
let mappingCompleted = false;
let mappingStoppedByUser = false;

const btnStartMapping = document.getElementById("btnStartMapping");
const btnManual = document.getElementById("btnManual");
const btnAuto = document.getElementById("btnAuto");
const btnSendObjectCommand = document.getElementById("btnSendObjectCommand");

const startMappingService = new ROSLIB.Service({
  ros: ros,
  name: "/task_manager/start_mapping",
  serviceType: "std_srvs/srv/Trigger"
});

const stopMappingService = new ROSLIB.Service({
  ros: ros,
  name: "/task_manager/stop_mapping",
  serviceType: "std_srvs/srv/Trigger"
});

const cancelObjectTaskService = new ROSLIB.Service({
  ros: ros,
  name: "/object_task_executor/cancel_task",
  serviceType: "std_srvs/srv/Trigger"
});

function callTriggerService(service) {
  return new Promise((resolve) => {
    const timeout = window.setTimeout(() => {
      resolve({
        result: false,
        success: false,
        message: "Keine Antwort vom Service"
      });
    }, 8000);

    service.callService({}, (response, result) => {
      window.clearTimeout(timeout);
      resolve({
        result,
        success: Boolean(response && response.success),
        message: response ? response.message : ""
      });
    });
  });
}

function updateActionButtons() {
  const autoMode = !manualMode;

  btnStartMapping.disabled = !autoMode || mappingActive || mappingCompleted;
  btnStartMapping.classList.toggle("hidden", mappingActive || mappingCompleted);

  btnSendObjectCommand.disabled = !autoMode;
}

btnStartMapping.addEventListener("click", async () => {
  if (manualMode || mappingActive || mappingCompleted) return;

  btnStartMapping.disabled = true;
  const response = await callTriggerService(startMappingService);

  if (response.success) {
    mappingActive = true;
    mappingStoppedByUser = false;
  } else {
    alert("Mapping konnte nicht gestartet werden: " + response.message);
  }

  updateActionButtons();
});

const taskManagerStatusTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/task_manager/status_text",
  messageType: "std_msgs/String"
});

taskManagerStatusTopic.subscribe((msg) => {
  const mappingRunning = /^mapping:\s+running\b/m.test(msg.data);

  if (mappingActive && !mappingRunning && !mappingStoppedByUser) {
    mappingCompleted = true;
  }

  mappingActive = mappingRunning;

  if (!mappingRunning && mappingStoppedByUser) {
    mappingStoppedByUser = false;
  }

  updateActionButtons();
});

const speedSlider = document.getElementById("speedSlider");
const speedValue = document.getElementById("speedValue");

speedSlider.addEventListener("input", () => {
  speedPercent = Number(speedSlider.value);
  speedValue.innerText = speedPercent + "%";
});

const cmdVelTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/cmd_vel",
  messageType: "geometry_msgs/Twist"
});

function sendCmdVel(linearX, angularZ) {
  if (!manualMode) return;

  const factor = speedPercent / 100.0;

  const twist = new ROSLIB.Message({
    linear: {
      x: linearX * factor,
      y: 0.0,
      z: 0.0
    },
    angular: {
      x: 0.0,
      y: 0.0,
      z: angularZ * factor
    }
  });

  cmdVelTopic.publish(twist);
}

document.getElementById("btnForward").addEventListener("click", () => {
  sendCmdVel(0.5, 0.0);
});

document.getElementById("btnBackward").addEventListener("click", () => {
  sendCmdVel(-0.5, 0.0);
});

document.getElementById("btnLeft").addEventListener("click", () => {
  sendCmdVel(0.0, 1.0);
});

document.getElementById("btnRight").addEventListener("click", () => {
  sendCmdVel(0.0, -1.0);
});

document.getElementById("btnStop").addEventListener("click", async () => {
  sendCmdVel(0.0, 0.0);
  await stopActiveWork();
});

// ----------------------------------------------------
// Modus: /robot_mode std_msgs/String
// ----------------------------------------------------

const modeTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/robot_mode",
  messageType: "std_msgs/String"
});

async function stopActiveWork() {
  await callTriggerService(cancelObjectTaskService);

  if (mappingActive) {
    mappingStoppedByUser = true;
    await callTriggerService(stopMappingService);
    mappingActive = false;
  }

  updateActionButtons();
}

btnManual.addEventListener("click", async () => {
  await stopActiveWork();

  manualMode = true;

  btnManual.classList.add("active");
  btnAuto.classList.remove("active");

  modeTopic.publish(new ROSLIB.Message({
    data: "manual"
  }));

  updateActionButtons();
});

btnAuto.addEventListener("click", () => {
  sendCmdVel(0.0, 0.0);

  manualMode = false;

  btnAuto.classList.add("active");
  btnManual.classList.remove("active");

  modeTopic.publish(new ROSLIB.Message({
    data: "auto"
  }));

  updateActionButtons();
});

updateActionButtons();

// ----------------------------------------------------
// Objekt- und Ecken-Auswahl
// ----------------------------------------------------

let availableObjects = [];
let availableCorners = [];

let selectedObject = null;
let selectedCorner = null;

const objectTileGrid = document.getElementById("objectTileGrid");
const cornerTileGrid = document.getElementById("cornerTileGrid");

// ----------------------------------------------------
// Objekte: /objects vision_msgs/Detection3DArray
// ----------------------------------------------------

const objectsTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/objects",
  messageType: "vision_msgs/Detection3DArray"
});

objectsTopic.subscribe((msg) => {
  availableObjects = msg.detections.map((detection, index) => {
    let name = "unknown";

    if (
      detection.results &&
      detection.results.length > 0 &&
      detection.results[0].hypothesis
    ) {
      name = detection.results[0].hypothesis.class_id;
    }

    return {
      id: index,
      name: name
    };
  });

  renderObjectTiles();
});

function renderObjectTiles() {
  objectTileGrid.innerHTML = "";

  if (availableObjects.length === 0) {
    objectTileGrid.innerHTML =
      `<div class="empty-info">Keine Objekte verfügbar</div>`;
    return;
  }

  availableObjects.forEach((obj) => {
    const tile = document.createElement("div");
    tile.className = "select-tile";

    if (selectedObject && selectedObject.id === obj.id) {
      tile.classList.add("selected");
    }

    tile.innerHTML = `
      <div class="object-icon">${getObjectIcon(obj.name)}</div>
      <div>${obj.name}</div>
      <small>ID: ${obj.id}</small>
    `;

    tile.addEventListener("click", () => {
      selectedObject = obj;
      renderObjectTiles();
    });

    objectTileGrid.appendChild(tile);
  });
}

function getObjectIcon(name) {
  const objectName = String(name).toLowerCase();

  if (objectName.includes("cube") || objectName.includes("würfel")) {
    return "🧊";
  }

  if (objectName.includes("ball") || objectName.includes("kugel")) {
    return "⚽";
  }

  if (objectName.includes("bottle") || objectName.includes("flasche")) {
    return "🧴";
  }

  if (objectName.includes("box") || objectName.includes("kiste")) {
    return "📦";
  }

  if (objectName.includes("cone") || objectName.includes("kegel")) {
    return "🔺";
  }

  if (objectName.includes("cylinder") || objectName.includes("zylinder")) {
    return "🥫";
  }

  return "⬛";
}

// ----------------------------------------------------
// Ecken: /corners/json std_msgs/String
//
// Erwarteter JSON-Inhalt:
// {
//   "corners": [
//     {
//       "corner_uid": "corner_1",
//       "corner_id": "red_corner",
//       "median_h": 0,
//       "median_s": 255,
//       "median_v": 255
//     }
//   ]
// }
// ----------------------------------------------------

const cornersJsonTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/corners/json",
  messageType: "std_msgs/String"
});

cornersJsonTopic.subscribe((msg) => {
  try {
    const data = JSON.parse(msg.data);
    availableCorners = data.corners || [];
    renderCornerTiles();
  } catch (e) {
    console.error("Fehler beim Lesen von /corners/json:", e);
  }
});

function renderCornerTiles() {
  cornerTileGrid.innerHTML = "";

  if (availableCorners.length === 0) {
    cornerTileGrid.innerHTML =
      `<div class="empty-info">Keine Ecken verfügbar</div>`;
    return;
  }

  availableCorners.forEach((corner) => {
    const tile = document.createElement("div");
    tile.className = "select-tile";

    if (
      selectedCorner &&
      selectedCorner.corner_uid === corner.corner_uid
    ) {
      tile.classList.add("selected");
    }

    const rgb = hsvToRgb(
      corner.median_h,
      corner.median_s,
      corner.median_v
    );

    tile.innerHTML = `
      <div
        class="corner-icon"
        style="background: rgb(${rgb.r}, ${rgb.g}, ${rgb.b});"
      ></div>
      <div>${corner.corner_id}</div>
      <small>${corner.corner_uid}</small>
    `;

    tile.addEventListener("click", () => {
      selectedCorner = corner;
      renderCornerTiles();
    });

    cornerTileGrid.appendChild(tile);
  });
}

// OpenCV-HSV:
// H = 0 bis 179
// S = 0 bis 255
// V = 0 bis 255

function hsvToRgb(h, s, v) {
  h = Number(h);
  s = Number(s);
  v = Number(v);

  h = (h / 179.0) * 360.0;
  s = s / 255.0;
  v = v / 255.0;

  const c = v * s;
  const x = c * (1.0 - Math.abs((h / 60.0) % 2.0 - 1.0));
  const m = v - c;

  let r1 = 0.0;
  let g1 = 0.0;
  let b1 = 0.0;

  if (h >= 0 && h < 60) {
    r1 = c;
    g1 = x;
    b1 = 0;
  } else if (h >= 60 && h < 120) {
    r1 = x;
    g1 = c;
    b1 = 0;
  } else if (h >= 120 && h < 180) {
    r1 = 0;
    g1 = c;
    b1 = x;
  } else if (h >= 180 && h < 240) {
    r1 = 0;
    g1 = x;
    b1 = c;
  } else if (h >= 240 && h < 300) {
    r1 = x;
    g1 = 0;
    b1 = c;
  } else {
    r1 = c;
    g1 = 0;
    b1 = x;
  }

  return {
    r: Math.round((r1 + m) * 255),
    g: Math.round((g1 + m) * 255),
    b: Math.round((b1 + m) * 255)
  };
}

// ----------------------------------------------------
// Auftrag an Controller:
// /object_place_command interfaces/ObjectPlaceCommand
// ----------------------------------------------------

const objectCommandTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/object_place_command",
  messageType: "interfaces/ObjectPlaceCommand"
});

btnSendObjectCommand.addEventListener("click", () => {
  if (manualMode) {
    alert("Auftrag kann nur im Automatikmodus gesendet werden.");
    return;
  }

  if (!selectedObject || !selectedCorner) {
    alert("Bitte Objekt und Ziel-Ecke auswählen.");
    return;
  }

  const msg = new ROSLIB.Message({
    header: {
      stamp: {
        sec: 0,
        nanosec: 0
      },
      frame_id: "map"
    },
    object_id: selectedObject.id,
    object_name: selectedObject.name,
    corner_uid: selectedCorner.corner_uid,
    mode: manualMode ? "manual" : "auto"
  });

  objectCommandTopic.publish(msg);
});
