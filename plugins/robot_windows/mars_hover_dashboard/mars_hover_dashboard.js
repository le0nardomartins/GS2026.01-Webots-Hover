var MOCK_MODE = true;
var MOCK_INTERVAL_MS = 700;
var HTTP_URL = "http://127.0.0.1:8765/telemetry";
var FILE_URL = "telemetry.json";

var lastTimestamp = null;
var lastDataAt = 0;
var pollTimer = null;
var mockTimer = null;

window.MARS_HOVER_DASHBOARD_BUILD = "mock-20260610";

window.receive = function (message) {
  var data = parseMessage(message);
  if (data) applyData(data);
};

function parseMessage(message) {
  try {
    if (typeof message === "string") return JSON.parse(message);
    if (message && typeof message.data === "string") return JSON.parse(message.data);
    if (message && typeof message === "object") return message;
  } catch (e) {
    setConnection(false, "JSON INVALIDO");
  }
  return null;
}

function poll() {
  var t = Date.now();

  readJSON(HTTP_URL + "?t=" + t)
    .catch(function () {
      return readJSON("http://localhost:8765/telemetry?t=" + t);
    })
    .catch(function () {
      return readJSON(FILE_URL + "?t=" + t);
    })
    .then(applyData)
    .catch(function () {
      if (!lastDataAt || Date.now() - lastDataAt > 1500) {
        setConnection(false, "SEM DADOS");
      }
    });
}

function readJSON(url) {
  if (window.fetch) {
    return fetch(url, { cache: "no-store" }).then(function (response) {
      if (!response.ok) throw new Error("HTTP " + response.status);
      return response.json();
    });
  }

  return readJSONWithXHR(url);
}

function readJSONWithXHR(url) {
  return new Promise(function (resolve, reject) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", url, true);
    xhr.onreadystatechange = function () {
      if (xhr.readyState !== 4) return;
      if ((xhr.status >= 200 && xhr.status < 300) || xhr.status === 0) {
        try {
          resolve(JSON.parse(xhr.responseText));
        } catch (e) {
          reject(e);
        }
      } else {
        reject(new Error("HTTP " + xhr.status));
      }
    };
    xhr.onerror = reject;
    xhr.send();
  });
}

function applyData(data) {
  if (!data || data.type !== "telemetry") return;

  var timestamp = data.timestamp || JSON.stringify(data);
  if (timestamp === lastTimestamp) {
    lastDataAt = Date.now();
    setConnection(true);
    return;
  }

  lastTimestamp = timestamp;
  lastDataAt = Date.now();
  updateDashboard(data);
}

function initWebotsWindow() {
  if (MOCK_MODE) {
    initMockTelemetry();
    return;
  }

  if (typeof webots !== "undefined" && webots.window) {
    var names = ["Mars_Hover", "mars_hover_controller", "mars_hover_dashboard"];
    for (var i = 0; i < names.length; i++) {
      try {
        var robotWindow = webots.window(names[i]);
        if (robotWindow) {
          robotWindow.receive = window.receive;
          break;
        }
      } catch (e) {
        // Some Webots versions expose only the global receive function.
      }
    }
  }

  poll();
  pollTimer = setInterval(poll, 250);
}

function initMockTelemetry() {
  if (window.console) console.log("Mars Hover dashboard running mock telemetry", window.MARS_HOVER_DASHBOARD_BUILD);
  setConnection(true, "MOCK");
  applyData(generateMockTelemetry());
  mockTimer = setInterval(function () {
    applyData(generateMockTelemetry());
  }, MOCK_INTERVAL_MS);
}

function generateMockTelemetry() {
  var proximity = generateMockProximity();
  var nav = deriveMockNavigation(proximity);
  var currents = generateMockCurrents(nav.risk);

  return {
    type: "telemetry",
    mock: true,
    state: nav.state,
    action: nav.action,
    risk: nav.risk,
    route: nav.route,
    obstacles: nav.obstacles,
    zones: nav.zones,
    proximity: proximity,
    currents: currents,
    totalCurrent: sumCurrents(currents),
    mode: chooseWeighted([
      ["AUTO", 0.78],
      ["MANUAL", 0.14],
      ["STOP", 0.08]
    ]),
    timestamp: Date.now()
  };
}

function generateMockProximity() {
  var scenario = chooseWeighted([
    ["clear", 0.36],
    ["warning", 0.28],
    ["frontBlock", 0.2],
    ["leftBlock", 0.08],
    ["rightBlock", 0.08]
  ]);

  var p = {
    prox_front: rand(0, 280),
    prox_front_left: rand(0, 260),
    prox_front_right: rand(0, 260),
    prox_left: rand(0, 220),
    prox_right: rand(0, 220)
  };

  if (scenario === "warning") {
    p[choose(["prox_front", "prox_front_left", "prox_front_right"])] = rand(450, 720);
  } else if (scenario === "frontBlock") {
    p.prox_front = rand(760, 1000);
    p.prox_front_left = rand(120, 760);
    p.prox_front_right = rand(120, 760);
  } else if (scenario === "leftBlock") {
    p.prox_front_left = rand(760, 1000);
    p.prox_left = rand(520, 900);
  } else if (scenario === "rightBlock") {
    p.prox_front_right = rand(760, 1000);
    p.prox_right = rand(520, 900);
  }

  return p;
}

function deriveMockNavigation(proximity) {
  var leftLoad = Math.max(proximity.prox_front_left, proximity.prox_left);
  var rightLoad = Math.max(proximity.prox_front_right, proximity.prox_right);
  var front = proximity.prox_front;

  var zones = {
    left: leftLoad >= 650 ? "Ocupada" : "Livre",
    center: front >= 650 ? "Ocupada" : "Livre",
    right: rightLoad >= 650 ? "Ocupada" : "Livre"
  };

  var obstacles = 0;
  if (zones.left === "Ocupada") obstacles++;
  if (zones.center === "Ocupada") obstacles++;
  if (zones.right === "Ocupada") obstacles++;

  if (front >= 880 && leftLoad >= 760 && rightLoad >= 760) {
    return {
      state: "PARAR",
      action: "PARADA DE SEGURANCA",
      risk: "CRITICO",
      route: "BLOQUEADA",
      obstacles: obstacles,
      zones: zones
    };
  }

  if (front >= 650 || zones.center === "Ocupada") {
    var goLeft = leftLoad <= rightLoad;
    return {
      state: goLeft ? "DESVIAR_ESQ" : "DESVIAR_DIR",
      action: goLeft ? "DESVIAR PARA ESQUERDA" : "DESVIAR PARA DIREITA",
      risk: front >= 760 ? "CRITICO" : "ALTO",
      route: goLeft ? "ESQUERDA LIVRE" : "DIREITA LIVRE",
      obstacles: Math.max(1, obstacles),
      zones: zones
    };
  }

  if (leftLoad >= 650 || rightLoad >= 650) {
    return {
      state: leftLoad > rightLoad ? "DESVIAR_DIR" : "DESVIAR_ESQ",
      action: leftLoad > rightLoad ? "CORRIGIR PARA DIREITA" : "CORRIGIR PARA ESQUERDA",
      risk: "ALTO",
      route: leftLoad > rightLoad ? "BORDA ESQUERDA OCUPADA" : "BORDA DIREITA OCUPADA",
      obstacles: Math.max(1, obstacles),
      zones: zones
    };
  }

  if (front >= 450 || leftLoad >= 450 || rightLoad >= 450) {
    return {
      state: "ALERTA",
      action: "REDUZIR VELOCIDADE",
      risk: "MEDIO",
      route: "MONITORANDO OBSTACULO",
      obstacles: Math.max(1, obstacles),
      zones: zones
    };
  }

  return {
    state: "LIVRE",
    action: "AVANCAR",
    risk: "BAIXO",
    route: "ROTA CENTRAL",
    obstacles: 0,
    zones: zones
  };
}

function generateMockCurrents(risk) {
  var base = risk === "CRITICO" ? [34, 58] : risk === "ALTO" ? [24, 46] : risk === "MEDIO" ? [14, 32] : [5, 18];
  return {
    motor_front_left: rand(base[0], base[1]),
    motor_mid_left: rand(base[0], base[1]),
    motor_back_left: rand(base[0], base[1]),
    motor_front_right: rand(base[0], base[1]),
    motor_mid_right: rand(base[0], base[1]),
    motor_back_right: rand(base[0], base[1])
  };
}

function sumCurrents(currents) {
  var total = 0;
  for (var key in currents) {
    if (Object.prototype.hasOwnProperty.call(currents, key)) total += Number(currents[key] || 0);
  }
  return total;
}

function rand(min, max) {
  return min + Math.random() * (max - min);
}

function choose(items) {
  return items[Math.floor(Math.random() * items.length)];
}

function chooseWeighted(items) {
  var roll = Math.random();
  var acc = 0;
  for (var i = 0; i < items.length; i++) {
    acc += items[i][1];
    if (roll <= acc) return items[i][0];
  }
  return items[items.length - 1][0];
}

function sendCommand(command) {
  var msg = JSON.stringify({ type: "command", command: command });
  if (typeof window.send === "function") window.send(msg);

  if (typeof webots !== "undefined" && webots.window) {
    try {
      var robotWindow = webots.window("Mars_Hover");
      if (robotWindow && typeof robotWindow.send === "function") robotWindow.send(msg);
    } catch (e) {}
  }
}

function updateDashboard(data) {
  setConnection(true, data.mock ? "MOCK" : undefined);

  setText("roverState", data.state || "---");
  setText("roverAction", data.action || "---");
  setText("roverRisk", data.risk || "---");
  setText("roverRoute", data.route || "---");
  setText("obstacleCount", String(data.obstacles !== undefined ? data.obstacles : 0));
  setText("totalCurrent", fmt(data.totalCurrent) + " A");
  setText("lastUpdate", new Date().toLocaleTimeString());

  setZone("zoneLeft", "zoneCellLeft", (data.zones && data.zones.left) || "Livre");
  setZone("zoneCenter", "zoneCellCenter", (data.zones && data.zones.center) || "Livre");
  setZone("zoneRight", "zoneCellRight", (data.zones && data.zones.right) || "Livre");

  updateStateColor(data.state || "");
  updateMode(data.mode || "");

  if (data.currents) {
    setMotor("motor_front_left", data.currents.motor_front_left);
    setMotor("motor_mid_left", data.currents.motor_mid_left);
    setMotor("motor_back_left", data.currents.motor_back_left);
    setMotor("motor_front_right", data.currents.motor_front_right);
    setMotor("motor_mid_right", data.currents.motor_mid_right);
    setMotor("motor_back_right", data.currents.motor_back_right);
  }

  if (data.proximity) {
    setProximity("prox_front", "F", data.proximity.prox_front);
    setProximity("prox_front_left", "FL", data.proximity.prox_front_left);
    setProximity("prox_front_right", "FR", data.proximity.prox_front_right);
    setProximity("prox_left", "L", data.proximity.prox_left);
    setProximity("prox_right", "R", data.proximity.prox_right);
  }
}

function setConnection(on, label) {
  var status = document.getElementById("connectionStatus");
  var text = document.getElementById("statusText");
  if (!status) return;
  status.classList.toggle("connected", on);
  status.classList.toggle("disconnected", !on);
  if (text) text.textContent = label || (on ? "CONECTADO" : "AGUARDANDO");
}

function updateStateColor(state) {
  var el = document.getElementById("roverState");
  if (!el) return;
  el.classList.remove("state-livre", "state-alerta", "state-parar", "state-desviar");
  if (state === "LIVRE") el.classList.add("state-livre");
  else if (state === "ALERTA") el.classList.add("state-alerta");
  else if (state === "PARAR") el.classList.add("state-parar");
  else if (state.indexOf("DESVIAR") >= 0) el.classList.add("state-desviar");
}

function updateMode(mode) {
  var el = document.getElementById("currentMode");
  if (!el) return;
  el.textContent = mode || "---";
  el.classList.remove("mode-auto", "mode-manual", "mode-stop");
  if (mode === "AUTO") el.classList.add("mode-auto");
  else if (mode === "MANUAL") el.classList.add("mode-manual");
  else if (mode === "STOP") el.classList.add("mode-stop");
}

function setZone(textId, cellId, value) {
  var textEl = document.getElementById(textId);
  var cellEl = document.getElementById(cellId);
  if (textEl) textEl.textContent = value;
  if (cellEl) cellEl.classList.toggle("occupied", value === "Ocupada");
}

function setMotor(id, value) {
  var current = Number(value || 0);
  var textEl = document.getElementById(id);
  var barEl = document.getElementById("bar_" + id);
  if (textEl) textEl.textContent = fmt(current) + " A";
  if (barEl) barEl.style.width = Math.min(100, (current / 60) * 100) + "%";
}

function setProximity(id, label, value) {
  var v = Number(value || 0);
  var el = document.getElementById(id);
  if (!el) return;
  el.textContent = label + ": " + v.toFixed(0);
  el.classList.remove("warning", "danger");
  if (v >= 750) el.classList.add("danger");
  else if (v >= 450) el.classList.add("warning");
}

function setText(id, value) {
  var el = document.getElementById(id);
  if (el) el.textContent = value;
}

function fmt(value) {
  return Number(value || 0).toFixed(1);
}

function initFullscreen() {
  var btn = document.getElementById("btnFullscreen");
  if (!btn) return;

  btn.addEventListener("click", function () {
    var canFullscreen = document.documentElement.requestFullscreen || document.documentElement.webkitRequestFullscreen;

    if (canFullscreen) {
      if (!document.fullscreenElement && !document.webkitFullscreenElement) {
        var request = document.documentElement.requestFullscreen || document.documentElement.webkitRequestFullscreen;
        var result = request.call(document.documentElement);
        if (result && typeof result.catch === "function") {
          result.catch(function () {
            toggleMaximized(btn);
          });
        }
      } else {
        var exit = document.exitFullscreen || document.webkitExitFullscreen;
        if (exit) exit.call(document);
      }
      return;
    }

    toggleMaximized(btn);
  });

  document.addEventListener("fullscreenchange", syncFullscreenButton);
  document.addEventListener("webkitfullscreenchange", syncFullscreenButton);
}

function toggleMaximized(btn) {
  document.body.classList.toggle("window-maximized");
  btn.classList.toggle("active", document.body.classList.contains("window-maximized"));
}

function syncFullscreenButton() {
  var btn = document.getElementById("btnFullscreen");
  if (!btn) return;
  var active = !!(document.fullscreenElement || document.webkitFullscreenElement);
  btn.classList.toggle("active", active);
}

window.addEventListener("load", function () {
  initWebotsWindow();
  initFullscreen();
});

window.addEventListener("beforeunload", function () {
  if (pollTimer) clearInterval(pollTimer);
  if (mockTimer) clearInterval(mockTimer);
});
