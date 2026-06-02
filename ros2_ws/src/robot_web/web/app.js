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
// ROS Logs: /rosout rcl_interfaces/Log
// Shows WARN, ERROR and FATAL in the dashboard.
// ----------------------------------------------------

const ROS_LOG_LEVELS = {
  10: "DEBUG",
  20: "INFO",
  30: "WARN",
  40: "ERROR",
  50: "FATAL"
};
const MAX_LOG_ENTRIES = 120;

const rosLogConsole = document.getElementById("rosLogConsole");
const btnClearLogs = document.getElementById("btnClearLogs");

const rosoutTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/rosout",
  messageType: "rcl_interfaces/Log"
});

rosoutTopic.subscribe((msg) => {
  if (msg.level < 30) return;
  appendRosLog(msg);
});

[
  { name: "/camera/status_text", node: "camera_node" },
  { name: "/esp32_drive/diagnostics", node: "esp32_drive" },
  { name: "/esp32_gripper/diagnostics", node: "esp32_gripper" }
].forEach((source) => {
  const topic = new ROSLIB.Topic({
    ros: ros,
    name: source.name,
    messageType: "std_msgs/String"
  });

  topic.subscribe((msg) => {
    appendTextLog(source.node, msg.data || "");
  });
});

btnClearLogs.addEventListener("click", () => {
  rosLogConsole.replaceChildren();
});

function appendRosLog(msg) {
  const level = ROS_LOG_LEVELS[msg.level] || String(msg.level);
  appendLogEntry(level, msg.name || "-", msg.msg || "", formatRosStamp(msg.stamp));
}

function appendTextLog(nodeName, text) {
  const match = String(text).match(/^(DEBUG|INFO|WARN|ERROR|FATAL)\s+(.*)$/);
  const level = match ? match[1] : "INFO";
  const message = match ? match[2] : String(text);
  if (level === "DEBUG" || level === "INFO") return;
  appendLogEntry(level, nodeName, message, new Date().toLocaleTimeString());
}

function appendLogEntry(level, nodeName, text, stampText) {
  const entry = document.createElement("div");
  entry.className = `log-entry ${level.toLowerCase()}`;

  const time = document.createElement("span");
  time.className = "log-time";
  time.textContent = stampText;

  const levelNode = document.createElement("span");
  levelNode.className = "log-level";
  levelNode.textContent = level;

  const node = document.createElement("span");
  node.className = "log-node";
  node.title = nodeName;
  node.textContent = nodeName;

  const message = document.createElement("span");
  message.className = "log-message";
  message.textContent = text;

  entry.append(time, levelNode, node, message);
  rosLogConsole.appendChild(entry);

  while (rosLogConsole.children.length > MAX_LOG_ENTRIES) {
    rosLogConsole.firstElementChild.remove();
  }

  rosLogConsole.scrollTop = rosLogConsole.scrollHeight;
}

function formatRosStamp(stamp) {
  if (!stamp || typeof stamp.sec !== "number") {
    return new Date().toLocaleTimeString();
  }

  const ms = stamp.sec * 1000 + Math.floor((stamp.nanosec || 0) / 1000000);
  return new Date(ms).toLocaleTimeString();
}

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
const btnGripOpen = document.getElementById("btnGripOpen");
const btnGripClose = document.getElementById("btnGripClose");
const btnLiftUp = document.getElementById("btnLiftUp");
const btnLiftDown = document.getElementById("btnLiftDown");
const btnGripperStop = document.getElementById("btnGripperStop");
const serviceControlList = document.getElementById("serviceControlList");

const TASK_MANAGER_TARGETS = [
  { id: "description", label: "Description" },
  { id: "hardware", label: "Hardware" },
  { id: "odometry", label: "Odometry" },
  { id: "slam", label: "SLAM" },
  { id: "navigation_debug", label: "Nav Debug" },
  { id: "navigation_slam", label: "Nav SLAM" },
  { id: "frontier_explorer", label: "Frontier" },
  { id: "vision", label: "Vision" }
];

const taskManagerServices = new Map();
const taskManagerRows = new Map();
const taskManagerStates = new Map();

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

function callTriggerService(service, timeoutMs = 15000) {
  return new Promise((resolve) => {
    const timeout = window.setTimeout(() => {
      resolve({
        result: false,
        success: false,
        message: "Keine Antwort vom Service"
      });
    }, timeoutMs);

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

function getTaskManagerService(action, target) {
  const key = `${action}:${target}`;
  if (!taskManagerServices.has(key)) {
    taskManagerServices.set(key, new ROSLIB.Service({
      ros: ros,
      name: `/task_manager/${action}_${target}`,
      serviceType: "std_srvs/srv/Trigger"
    }));
  }

  return taskManagerServices.get(key);
}

function renderServiceControls() {
  if (!serviceControlList) return;

  serviceControlList.replaceChildren();

  TASK_MANAGER_TARGETS.forEach((target) => {
    const row = document.createElement("div");
    row.className = "service-row";

    const name = document.createElement("span");
    name.className = "service-name";
    name.textContent = target.label;

    const state = document.createElement("span");
    state.className = "service-state stopped";
    state.textContent = "stopped";

    const startButton = document.createElement("button");
    startButton.type = "button";
    startButton.textContent = "Start";

    const stopButton = document.createElement("button");
    stopButton.type = "button";
    stopButton.textContent = "Stop";
    stopButton.className = "danger";

    startButton.addEventListener("click", () => runTaskManagerAction("start", target.id));
    stopButton.addEventListener("click", () => runTaskManagerAction("stop", target.id));

    row.append(name, state, startButton, stopButton);
    serviceControlList.appendChild(row);
    taskManagerRows.set(target.id, { row, state, startButton, stopButton });
  });
}

async function runTaskManagerAction(action, target) {
  const row = taskManagerRows.get(target);
  if (row) {
    row.startButton.disabled = true;
    row.stopButton.disabled = true;
    row.state.textContent = action === "start" ? "starting" : "stopping";
    row.state.className = "service-state pending";
  }

  const response = await callTriggerService(
    getTaskManagerService(action, target),
    target === "navigation_slam" || target === "navigation_debug" ? 90000 : 30000
  );

  if (!response.success) {
    alert(`${target} ${action} fehlgeschlagen: ${response.message}`);
  }

  updateServiceControls();
}

function parseTaskManagerStatus(statusText) {
  taskManagerStates.clear();

  String(statusText || "").split("\n").forEach((line) => {
    const match = line.match(/^([^:]+):\s+(\w+)/);
    if (!match) return;
    taskManagerStates.set(match[1], match[2]);
  });
}

function updateServiceControls() {
  TASK_MANAGER_TARGETS.forEach((target) => {
    const row = taskManagerRows.get(target.id);
    if (!row) return;

    const state = taskManagerStates.get(target.id) || "unknown";
    const running = state === "running";
    const exited = state === "exited";

    row.state.textContent = state;
    row.state.className = `service-state ${running ? "running" : exited ? "exited" : "stopped"}`;
    row.startButton.disabled = running;
    row.stopButton.disabled = !running;
  });
}

function updateActionButtons() {
  const autoMode = !manualMode;

  btnStartMapping.disabled = !autoMode || mappingActive || mappingCompleted;
  btnStartMapping.classList.toggle("hidden", mappingActive || mappingCompleted);

  btnSendObjectCommand.disabled = !autoMode;

  [btnGripOpen, btnGripClose, btnLiftUp, btnLiftDown, btnGripperStop].forEach((button) => {
    button.disabled = !manualMode;
  });
}

btnStartMapping.addEventListener("click", async () => {
  if (manualMode || mappingActive || mappingCompleted) return;

  btnStartMapping.disabled = true;
  const response = await callTriggerService(startMappingService, 300000);

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
  parseTaskManagerStatus(msg.data);
  updateServiceControls();

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

renderServiceControls();

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

const gripperCommandTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/esp32_gripper/command",
  messageType: "std_msgs/Float32"
});

const gripperManualTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/esp32_gripper/manual",
  messageType: "std_msgs/Int32"
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
  sendGripperManual(0);
  await stopActiveWork();
});

function sendGripperGap(gapMeters) {
  if (!manualMode) return;
  gripperCommandTopic.publish(new ROSLIB.Message({
    data: gapMeters
  }));
}

function sendGripperManual(command) {
  if (!manualMode) return;
  gripperManualTopic.publish(new ROSLIB.Message({
    data: command
  }));
}

btnGripOpen.addEventListener("click", () => {
  sendGripperGap(0.08);
});

btnGripClose.addEventListener("click", () => {
  sendGripperGap(0.0);
});

bindHoldButton(btnLiftUp, 3);
bindHoldButton(btnLiftDown, 4);

btnGripperStop.addEventListener("click", () => {
  sendGripperManual(0);
});

function bindHoldButton(button, command) {
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    sendGripperManual(command);
  });

  ["pointerup", "pointerleave", "pointercancel"].forEach((eventName) => {
    button.addEventListener(eventName, () => {
      sendGripperManual(0);
    });
  });
}

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
