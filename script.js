const MASS = 0.05;
const DEFAULT_AREA = 0.0008;
const THRUST = 12;
const THRUST_DURATION = 0.08;
const RHO = 1.225;
const TRACK_LENGTH = 20;
const SIM_DT = 0.01;
const MAX_SIM_TIME = 8;
const MAX_EDGE_COUNT = 1400;

const dragState = {
  manualCd: 0.32,
  externalCd: null,
};

const areaState = {
  manualArea: DEFAULT_AREA,
};

const defaultModel = createDefaultModel();

const modelState = {
  name: defaultModel.name,
  bounds: defaultModel.bounds,
  topProfile: defaultModel.topProfile,
  edgeSegments: defaultModel.edgeSegments,
  estimatedFrontalArea: DEFAULT_AREA,
  loaded: false,
};

const state = {
  running: false,
  finished: false,
  time: 0,
  distance: 0,
  speed: 0,
  acceleration: 0,
  dragForce: 0,
  thrustForce: 0,
  finishTime: null,
  accumulator: 0,
  lastFrameMs: null,
  telemetry: [createTelemetrySample(0, 0, 0, 0, 0, 0)],
};

const dom = {
  carFileInput: document.getElementById("carFileInput"),
  fileStatus: document.getElementById("fileStatus"),
  modelName: document.getElementById("modelName"),
  modelDimensions: document.getElementById("modelDimensions"),
  modelArea: document.getElementById("modelArea"),
  cdSlider: document.getElementById("cdSlider"),
  cdInput: document.getElementById("cdInput"),
  useManualCd: document.getElementById("useManualCd"),
  areaInput: document.getElementById("areaInput"),
  useModelArea: document.getElementById("useModelArea"),
  launchButton: document.getElementById("launchButton"),
  resetButton: document.getElementById("resetButton"),
  cdSource: document.getElementById("cdSource"),
  finishTime: document.getElementById("finishTime"),
  speedValue: document.getElementById("speedValue"),
  timeValue: document.getElementById("timeValue"),
  distanceValue: document.getElementById("distanceValue"),
  dragValue: document.getElementById("dragValue"),
  accelValue: document.getElementById("accelValue"),
  thrustValue: document.getElementById("thrustValue"),
  trackCanvas: document.getElementById("trackCanvas"),
  sideTrackCanvas: document.getElementById("sideTrackCanvas"),
  speedGraphCanvas: document.getElementById("speedGraphCanvas"),
  distanceGraphCanvas: document.getElementById("distanceGraphCanvas"),
  dragGraphCanvas: document.getElementById("dragGraphCanvas"),
  accelGraphCanvas: document.getElementById("accelGraphCanvas"),
};

const trackCtx = dom.trackCanvas.getContext("2d");
const sideTrackCtx = dom.sideTrackCanvas.getContext("2d");
const speedGraphCtx = dom.speedGraphCanvas.getContext("2d");
const distanceGraphCtx = dom.distanceGraphCanvas.getContext("2d");
const dragGraphCtx = dom.dragGraphCanvas.getContext("2d");
const accelGraphCtx = dom.accelGraphCanvas.getContext("2d");

function createTelemetrySample(time, speed, distance, drag, acceleration, thrust) {
  return { time, speed, distance, drag, acceleration, thrust };
}

function getDragCoefficient() {
  if (Number.isFinite(dragState.externalCd)) {
    return dragState.externalCd;
  }

  return dragState.manualCd;
}

function getReferenceArea() {
  return areaState.manualArea;
}

function setManualCd(value) {
  dragState.manualCd = clamp(Number(value) || 0.32, 0.05, 1.2);
  syncCdInputs(dragState.manualCd);
  updateCdSourceLabel();
}

function setExternalCd(value) {
  dragState.externalCd = Number.isFinite(value) ? clamp(value, 0.01, 5) : null;
  updateCdSourceLabel();
}

function setExternalCdFromCFD(results) {
  const value = results?.forces?.cd;
  if (Number.isFinite(value)) {
    setExternalCd(value);
    syncCdInputs(value);
  }
}

function clearExternalCd() {
  dragState.externalCd = null;
  updateCdSourceLabel();
}

function setReferenceArea(value) {
  areaState.manualArea = clamp(Number(value) || DEFAULT_AREA, 0.0001, 0.01);
  dom.areaInput.value = areaState.manualArea.toFixed(4);
}

window.TrackSimulationBridge = {
  getDragCoefficient,
  getReferenceArea,
  setManualCd,
  setExternalCd,
  setExternalCdFromCFD,
  clearExternalCd,
  setReferenceArea,
};

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function syncCdInputs(value) {
  const formatted = value.toFixed(3);
  dom.cdSlider.value = formatted;
  dom.cdInput.value = formatted;
}

function updateCdSourceLabel() {
  const cd = getDragCoefficient();
  const source = Number.isFinite(dragState.externalCd) ? "External CFD" : "Manual input";
  dom.cdSource.textContent = `${source} • Cd ${cd.toFixed(3)}`;
}

function resetSimulation() {
  state.running = false;
  state.finished = false;
  state.time = 0;
  state.distance = 0;
  state.speed = 0;
  state.acceleration = 0;
  state.dragForce = 0;
  state.thrustForce = 0;
  state.finishTime = null;
  state.accumulator = 0;
  state.lastFrameMs = null;
  state.telemetry = [createTelemetrySample(0, 0, 0, 0, 0, 0)];
  updateMetrics();
  render();
}

function launchSimulation() {
  resetSimulation();
  state.running = true;
}

function stepSimulation(dt) {
  const previousDistance = state.distance;
  const previousTime = state.time;
  const previousSpeed = state.speed;
  const cd = getDragCoefficient();
  const referenceArea = getReferenceArea();
  const thrustForce = state.time < THRUST_DURATION ? THRUST : 0;
  const dragForce = 0.5 * RHO * cd * referenceArea * state.speed * state.speed;
  const netForce = thrustForce - dragForce;
  const acceleration = netForce / MASS;
  const nextSpeed = Math.max(0, state.speed + acceleration * dt);
  const averageSpeed = (previousSpeed + nextSpeed) * 0.5;
  const nextDistance = state.distance + averageSpeed * dt;

  state.time += dt;
  state.speed = nextSpeed;
  state.distance = nextDistance;
  state.acceleration = acceleration;
  state.dragForce = dragForce;
  state.thrustForce = thrustForce;
  state.telemetry.push(createTelemetrySample(state.time, state.speed, state.distance, dragForce, acceleration, thrustForce));

  if (state.distance >= TRACK_LENGTH) {
    const segmentDistance = state.distance - previousDistance;
    const fraction = segmentDistance > 0 ? (TRACK_LENGTH - previousDistance) / segmentDistance : 1;
    state.finishTime = previousTime + clamp(fraction, 0, 1) * dt;
    state.distance = TRACK_LENGTH;
    state.running = false;
    state.finished = true;
  }

  if (state.time >= MAX_SIM_TIME && !state.finished) {
    state.running = false;
  }
}

function animate(frameMs) {
  if (state.lastFrameMs === null) {
    state.lastFrameMs = frameMs;
  }

  const elapsed = Math.min((frameMs - state.lastFrameMs) / 1000, 0.05);
  state.lastFrameMs = frameMs;

  if (state.running) {
    state.accumulator += elapsed;

    while (state.accumulator >= SIM_DT && state.running) {
      stepSimulation(SIM_DT);
      state.accumulator -= SIM_DT;
    }

    updateMetrics();
  }

  render();
  requestAnimationFrame(animate);
}

function updateMetrics() {
  dom.speedValue.textContent = `${state.speed.toFixed(2)} m/s`;
  dom.timeValue.textContent = `${state.time.toFixed(2)} s`;
  dom.distanceValue.textContent = `${state.distance.toFixed(2)} m`;
  dom.dragValue.textContent = `${state.dragForce.toFixed(3)} N`;
  dom.accelValue.textContent = `${state.acceleration.toFixed(2)} m/s²`;
  dom.thrustValue.textContent = `${state.thrustForce.toFixed(2)} N`;

  if (state.finished && state.finishTime !== null) {
    dom.finishTime.textContent = `${state.finishTime.toFixed(3)} s`;
  } else if (state.running) {
    dom.finishTime.textContent = "Racing...";
  } else {
    dom.finishTime.textContent = "Waiting";
  }
}

function resizeCanvas(canvas, context) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }

  context.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function render() {
  resizeCanvas(dom.trackCanvas, trackCtx);
  resizeCanvas(dom.sideTrackCanvas, sideTrackCtx);
  resizeCanvas(dom.speedGraphCanvas, speedGraphCtx);
  resizeCanvas(dom.distanceGraphCanvas, distanceGraphCtx);
  resizeCanvas(dom.dragGraphCanvas, dragGraphCtx);
  resizeCanvas(dom.accelGraphCanvas, accelGraphCtx);
  renderTrack();
  renderSideTrack();
  renderCharts();
}

function renderTrack() {
  const width = dom.trackCanvas.clientWidth;
  const height = dom.trackCanvas.clientHeight;

  trackCtx.clearRect(0, 0, width, height);

  const bgGradient = trackCtx.createLinearGradient(0, 0, width, height);
  bgGradient.addColorStop(0, "#08101c");
  bgGradient.addColorStop(1, "#14253b");
  trackCtx.fillStyle = bgGradient;
  trackCtx.fillRect(0, 0, width, height);

  const laneTop = height * 0.24;
  const laneHeight = height * 0.5;
  const margin = 56;
  const startX = margin;
  const finishX = width - margin;
  const usableWidth = finishX - startX;

  trackCtx.fillStyle = "rgba(255,255,255,0.05)";
  roundRect(trackCtx, margin - 18, laneTop - 32, usableWidth + 36, laneHeight + 64, 28);
  trackCtx.fill();

  const asphalt = trackCtx.createLinearGradient(0, laneTop, 0, laneTop + laneHeight);
  asphalt.addColorStop(0, "#23384f");
  asphalt.addColorStop(0.5, "#182739");
  asphalt.addColorStop(1, "#22384d");
  trackCtx.fillStyle = asphalt;
  roundRect(trackCtx, margin, laneTop, usableWidth, laneHeight, 22);
  trackCtx.fill();

  trackCtx.strokeStyle = "rgba(255,255,255,0.2)";
  trackCtx.setLineDash([24, 18]);
  trackCtx.lineWidth = 3;
  trackCtx.beginPath();
  trackCtx.moveTo(startX, laneTop + laneHeight / 2);
  trackCtx.lineTo(finishX, laneTop + laneHeight / 2);
  trackCtx.stroke();
  trackCtx.setLineDash([]);

  drawStartLine(startX, laneTop, laneHeight);
  drawFinishLine(finishX, laneTop, laneHeight);

  const progress = clamp(state.distance / TRACK_LENGTH, 0, 1);
  const carX = startX + usableWidth * progress;
  const carY = laneTop + laneHeight / 2;
  const carLength = Math.min(115, usableWidth * 0.135);
  const carWidth = laneHeight * 0.3;

  drawSpeedGlow(carX, carY, carLength, carWidth);
  drawUploadedCar(carX, carY, carLength, carWidth);

  trackCtx.fillStyle = "rgba(238,246,255,0.72)";
  trackCtx.font = '600 18px "Rajdhani", sans-serif';
  trackCtx.fillText("START", startX - 20, laneTop - 10);
  trackCtx.fillText("FINISH", finishX - 34, laneTop - 10);
}

function renderSideTrack() {
  const width = dom.sideTrackCanvas.clientWidth;
  const height = dom.sideTrackCanvas.clientHeight;

  sideTrackCtx.clearRect(0, 0, width, height);

  const bgGradient = sideTrackCtx.createLinearGradient(0, 0, width, height);
  bgGradient.addColorStop(0, "#08101c");
  bgGradient.addColorStop(1, "#13263b");
  sideTrackCtx.fillStyle = bgGradient;
  sideTrackCtx.fillRect(0, 0, width, height);

  const margin = 56;
  const startX = margin;
  const finishX = width - margin;
  const usableWidth = finishX - startX;
  const groundY = height * 0.72;
  const horizonY = height * 0.24;

  sideTrackCtx.strokeStyle = "rgba(127, 201, 255, 0.14)";
  sideTrackCtx.lineWidth = 1;
  sideTrackCtx.beginPath();
  sideTrackCtx.moveTo(0, horizonY);
  sideTrackCtx.lineTo(width, horizonY);
  sideTrackCtx.stroke();

  const groundGradient = sideTrackCtx.createLinearGradient(0, groundY - 24, 0, height);
  groundGradient.addColorStop(0, "#28445b");
  groundGradient.addColorStop(1, "#13222f");
  sideTrackCtx.fillStyle = groundGradient;
  roundRect(sideTrackCtx, margin - 22, groundY - 28, usableWidth + 44, 58, 18);
  sideTrackCtx.fill();

  sideTrackCtx.strokeStyle = "rgba(255,255,255,0.24)";
  sideTrackCtx.lineWidth = 3;
  sideTrackCtx.beginPath();
  sideTrackCtx.moveTo(startX, groundY);
  sideTrackCtx.lineTo(finishX, groundY);
  sideTrackCtx.stroke();

  drawSideStartLine(startX, groundY);
  drawSideFinishLine(finishX, groundY);

  const progress = clamp(state.distance / TRACK_LENGTH, 0, 1);
  const carX = startX + usableWidth * progress;
  const carY = groundY - 8;
  const carLength = Math.min(118, usableWidth * 0.14);
  const carHeight = Math.max(44, height * 0.22);

  drawSideSpeedGlow(carX, carY, carLength, carHeight);
  drawSideCar(carX, carY, carLength, carHeight);

  sideTrackCtx.fillStyle = "rgba(238,246,255,0.72)";
  sideTrackCtx.font = '600 18px "Rajdhani", sans-serif';
  sideTrackCtx.fillText("0 m", startX - 10, groundY + 34);
  sideTrackCtx.fillText("20 m", finishX - 16, groundY + 34);
}

function drawStartLine(x, laneTop, laneHeight) {
  trackCtx.fillStyle = "#49dcb1";
  trackCtx.fillRect(x - 4, laneTop - 10, 8, laneHeight + 20);
  trackCtx.fillStyle = "rgba(73,220,177,0.14)";
  trackCtx.fillRect(x - 18, laneTop - 16, 36, laneHeight + 32);
}

function drawFinishLine(x, laneTop, laneHeight) {
  const square = 14;
  for (let row = 0; row < Math.ceil((laneHeight + 16) / square); row += 1) {
    for (let col = 0; col < 2; col += 1) {
      trackCtx.fillStyle = (row + col) % 2 === 0 ? "#eef6ff" : "#0e1724";
      trackCtx.fillRect(x - 12 + col * square, laneTop - 8 + row * square, square, square);
    }
  }
}

function drawSideStartLine(x, groundY) {
  sideTrackCtx.fillStyle = "#49dcb1";
  sideTrackCtx.fillRect(x - 4, groundY - 56, 8, 60);
  sideTrackCtx.fillStyle = "rgba(73,220,177,0.14)";
  sideTrackCtx.fillRect(x - 18, groundY - 66, 36, 74);
}

function drawSideFinishLine(x, groundY) {
  const square = 12;
  for (let row = 0; row < 5; row += 1) {
    for (let col = 0; col < 2; col += 1) {
      sideTrackCtx.fillStyle = (row + col) % 2 === 0 ? "#eef6ff" : "#0e1724";
      sideTrackCtx.fillRect(x - 10 + col * square, groundY - 58 + row * square, square, square);
    }
  }
}

function drawSpeedGlow(carX, carY, carLength, carWidth) {
  const speedRatio = clamp(state.speed / 30, 0, 1);
  const plumeLength = 30 + speedRatio * 120;

  const glow = trackCtx.createLinearGradient(carX - plumeLength, carY, carX, carY);
  glow.addColorStop(0, "rgba(82, 213, 255, 0)");
  glow.addColorStop(1, "rgba(82, 213, 255, 0.35)");
  trackCtx.fillStyle = glow;
  trackCtx.beginPath();
  trackCtx.ellipse(carX - carLength * 0.42, carY, plumeLength, carWidth * 0.78, 0, 0, Math.PI * 2);
  trackCtx.fill();

  if (state.thrustForce > 0) {
    const exhaust = trackCtx.createLinearGradient(carX - 48, carY, carX - carLength * 0.38, carY);
    exhaust.addColorStop(0, "rgba(255, 111, 97, 0)");
    exhaust.addColorStop(0.5, "rgba(255, 209, 102, 0.8)");
    exhaust.addColorStop(1, "rgba(255, 111, 97, 0.95)");
    trackCtx.fillStyle = exhaust;
    trackCtx.beginPath();
    trackCtx.moveTo(carX - carLength * 0.45, carY - carWidth * 0.12);
    trackCtx.lineTo(carX - 56, carY);
    trackCtx.lineTo(carX - carLength * 0.45, carY + carWidth * 0.12);
    trackCtx.closePath();
    trackCtx.fill();
  }
}

function drawSideSpeedGlow(carX, carY, carLength, carHeight) {
  const speedRatio = clamp(state.speed / 30, 0, 1);
  const plumeLength = 34 + speedRatio * 124;

  const glow = sideTrackCtx.createLinearGradient(carX - plumeLength, carY, carX, carY);
  glow.addColorStop(0, "rgba(82, 213, 255, 0)");
  glow.addColorStop(1, "rgba(82, 213, 255, 0.35)");
  sideTrackCtx.fillStyle = glow;
  sideTrackCtx.beginPath();
  sideTrackCtx.ellipse(carX - carLength * 0.42, carY - carHeight * 0.2, plumeLength, carHeight * 0.42, 0, 0, Math.PI * 2);
  sideTrackCtx.fill();

  if (state.thrustForce > 0) {
    const exhaust = sideTrackCtx.createLinearGradient(carX - 48, carY, carX - carLength * 0.36, carY);
    exhaust.addColorStop(0, "rgba(255, 111, 97, 0)");
    exhaust.addColorStop(0.5, "rgba(255, 209, 102, 0.85)");
    exhaust.addColorStop(1, "rgba(255, 111, 97, 0.96)");
    sideTrackCtx.fillStyle = exhaust;
    sideTrackCtx.beginPath();
    sideTrackCtx.moveTo(carX - carLength * 0.45, carY - carHeight * 0.18);
    sideTrackCtx.lineTo(carX - 58, carY - carHeight * 0.08);
    sideTrackCtx.lineTo(carX - carLength * 0.45, carY + carHeight * 0.02);
    sideTrackCtx.closePath();
    sideTrackCtx.fill();
  }
}

function drawUploadedCar(carX, carY, carLength, carWidth) {
  const topProfile = modelState.topProfile;
  const edgeSegments = modelState.edgeSegments;

  if (!topProfile.length) {
    return;
  }

  trackCtx.save();
  trackCtx.translate(carX, carY);
  trackCtx.scale(carLength, carWidth);

  const bodyGradient = trackCtx.createLinearGradient(-0.5, 0, 0.5, 0);
  bodyGradient.addColorStop(0, "#c4f8e4");
  bodyGradient.addColorStop(0.55, "#69d8ff");
  bodyGradient.addColorStop(1, "#e6fbff");

  trackCtx.beginPath();
  topProfile.forEach((point, index) => {
    const x = point.x;
    const y = point.y;
    if (index === 0) {
      trackCtx.moveTo(x, y);
    } else {
      trackCtx.lineTo(x, y);
    }
  });
  trackCtx.closePath();
  trackCtx.fillStyle = bodyGradient;
  trackCtx.fill();
  trackCtx.strokeStyle = "rgba(5, 16, 27, 0.45)";
  trackCtx.lineWidth = 0.035;
  trackCtx.stroke();

  trackCtx.strokeStyle = "rgba(255, 255, 255, 0.18)";
  trackCtx.lineWidth = 0.012;
  edgeSegments.forEach((edge) => {
    trackCtx.beginPath();
    trackCtx.moveTo(edge.a.x, edge.a.y);
    trackCtx.lineTo(edge.b.x, edge.b.y);
    trackCtx.stroke();
  });

  trackCtx.fillStyle = "rgba(4, 12, 21, 0.45)";
  trackCtx.beginPath();
  trackCtx.ellipse(0.05, 0, 0.12, 0.2, 0, 0, Math.PI * 2);
  trackCtx.fill();

  trackCtx.restore();
}

function drawSideCar(carX, carY, carLength, carHeight) {
  const bounds = modelState.bounds;
  const lengthRatio = bounds.length / Math.max(bounds.length, bounds.height, 1e-9);
  const heightRatio = bounds.height / Math.max(bounds.length, bounds.height, 1e-9);
  const noseX = carX + carLength * 0.48;
  const tailX = carX - carLength * 0.48;
  const roofY = carY - carHeight * (0.24 + 0.32 * heightRatio);
  const floorY = carY;
  const canopyX = carX - carLength * (0.08 + 0.18 * (1 - lengthRatio));
  const rearDeckX = carX - carLength * 0.24;

  const bodyGradient = sideTrackCtx.createLinearGradient(tailX, carY, noseX, carY);
  bodyGradient.addColorStop(0, "#c4f8e4");
  bodyGradient.addColorStop(0.55, "#69d8ff");
  bodyGradient.addColorStop(1, "#e6fbff");
  sideTrackCtx.fillStyle = bodyGradient;

  sideTrackCtx.beginPath();
  sideTrackCtx.moveTo(tailX, floorY - carHeight * 0.12);
  sideTrackCtx.quadraticCurveTo(carX - carLength * 0.34, roofY, canopyX, roofY);
  sideTrackCtx.quadraticCurveTo(carX + carLength * 0.14, roofY + carHeight * 0.06, noseX, floorY - carHeight * 0.1);
  sideTrackCtx.quadraticCurveTo(carX + carLength * 0.34, floorY - carHeight * 0.02, noseX - carLength * 0.02, floorY);
  sideTrackCtx.lineTo(rearDeckX, floorY);
  sideTrackCtx.quadraticCurveTo(tailX + carLength * 0.06, floorY, tailX, floorY - carHeight * 0.12);
  sideTrackCtx.closePath();
  sideTrackCtx.fill();
  sideTrackCtx.strokeStyle = "rgba(5, 16, 27, 0.45)";
  sideTrackCtx.lineWidth = 2;
  sideTrackCtx.stroke();

  sideTrackCtx.strokeStyle = "rgba(255,255,255,0.22)";
  sideTrackCtx.lineWidth = 1.2;
  sideTrackCtx.beginPath();
  sideTrackCtx.moveTo(rearDeckX, floorY - carHeight * 0.02);
  sideTrackCtx.quadraticCurveTo(carX - carLength * 0.05, roofY + carHeight * 0.08, carX + carLength * 0.18, floorY - carHeight * 0.1);
  sideTrackCtx.stroke();

  const wheelRadius = carHeight * 0.14;
  const wheelY = floorY + wheelRadius * 0.18;
  drawWheel(carX - carLength * 0.22, wheelY, wheelRadius);
  drawWheel(carX + carLength * 0.2, wheelY, wheelRadius);
}

function drawWheel(x, y, radius) {
  sideTrackCtx.fillStyle = "rgba(4, 12, 21, 0.92)";
  sideTrackCtx.beginPath();
  sideTrackCtx.arc(x, y, radius, 0, Math.PI * 2);
  sideTrackCtx.fill();

  sideTrackCtx.strokeStyle = "rgba(196, 248, 228, 0.35)";
  sideTrackCtx.lineWidth = 1.5;
  sideTrackCtx.beginPath();
  sideTrackCtx.arc(x, y, radius * 0.55, 0, Math.PI * 2);
  sideTrackCtx.stroke();
}

function renderCharts() {
  renderTelemetryChart(speedGraphCtx, dom.speedGraphCanvas, {
    title: "Speed",
    units: "m/s",
    accessor: (sample) => sample.speed,
    lineColorA: "#49dcb1",
    lineColorB: "#7fc9ff",
    minMaxHint: 12,
  });

  renderTelemetryChart(distanceGraphCtx, dom.distanceGraphCanvas, {
    title: "Distance",
    units: "m",
    accessor: (sample) => sample.distance,
    lineColorA: "#ffd166",
    lineColorB: "#ff8c61",
    minMaxHint: TRACK_LENGTH,
  });

  renderTelemetryChart(dragGraphCtx, dom.dragGraphCanvas, {
    title: "Drag Force",
    units: "N",
    accessor: (sample) => sample.drag,
    lineColorA: "#ff6f61",
    lineColorB: "#ffd166",
    minMaxHint: 0.2,
  });

  renderTelemetryChart(accelGraphCtx, dom.accelGraphCanvas, {
    title: "Acceleration",
    units: "m/s²",
    accessor: (sample) => sample.acceleration,
    lineColorA: "#a5f06b",
    lineColorB: "#49dcb1",
    minMaxHint: THRUST / MASS,
  });
}

function renderTelemetryChart(context, canvas, config) {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  context.clearRect(0, 0, width, height);

  const padding = { top: 18, right: 22, bottom: 40, left: 52 };
  const plotWidth = width - padding.left - padding.right;
  const plotHeight = height - padding.top - padding.bottom;
  const samples = state.telemetry;
  const maxTime = Math.max(1.2, samples[samples.length - 1].time, state.finishTime || 0);
  const values = samples.map(config.accessor);
  const minValue = Math.min(0, ...values);
  const maxValue = Math.max(config.minMaxHint, ...values, 0.0001);
  const range = Math.max(0.0001, maxValue - minValue);

  context.fillStyle = "rgba(255,255,255,0.03)";
  roundRect(context, padding.left, padding.top, plotWidth, plotHeight, 16);
  context.fill();

  context.strokeStyle = "rgba(255,255,255,0.09)";
  context.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = padding.top + (plotHeight / 4) * i;
    context.beginPath();
    context.moveTo(padding.left, y);
    context.lineTo(padding.left + plotWidth, y);
    context.stroke();
  }

  context.beginPath();
  context.moveTo(padding.left, padding.top);
  context.lineTo(padding.left, padding.top + plotHeight);
  context.lineTo(padding.left + plotWidth, padding.top + plotHeight);
  context.strokeStyle = "rgba(255,255,255,0.28)";
  context.lineWidth = 2;
  context.stroke();

  context.save();
  context.beginPath();
  context.rect(padding.left, padding.top, plotWidth, plotHeight);
  context.clip();

  context.beginPath();
  samples.forEach((sample, index) => {
    const value = config.accessor(sample);
    const x = padding.left + (sample.time / maxTime) * plotWidth;
    const y = padding.top + plotHeight - ((value - minValue) / range) * plotHeight;
    if (index === 0) {
      context.moveTo(x, y);
    } else {
      context.lineTo(x, y);
    }
  });

  const graphGradient = context.createLinearGradient(padding.left, padding.top, padding.left + plotWidth, padding.top);
  graphGradient.addColorStop(0, config.lineColorA);
  graphGradient.addColorStop(1, config.lineColorB);
  context.strokeStyle = graphGradient;
  context.lineWidth = 3;
  context.stroke();

  context.lineTo(padding.left + (samples[samples.length - 1].time / maxTime) * plotWidth, padding.top + plotHeight);
  context.lineTo(padding.left, padding.top + plotHeight);
  context.closePath();

  const fillGradient = context.createLinearGradient(0, padding.top, 0, padding.top + plotHeight);
  fillGradient.addColorStop(0, `${hexToRgba(config.lineColorA, 0.22)}`);
  fillGradient.addColorStop(1, `${hexToRgba(config.lineColorB, 0.02)}`);
  context.fillStyle = fillGradient;
  context.fill();
  context.restore();

  context.fillStyle = "rgba(238,246,255,0.75)";
  context.font = '600 13px "Rajdhani", sans-serif';
  context.fillText("Time (s)", padding.left + plotWidth - 40, height - 12);

  context.save();
  context.translate(16, padding.top + plotHeight / 2);
  context.rotate(-Math.PI / 2);
  context.fillText(`${config.title} (${config.units})`, 0, 0);
  context.restore();

  context.fillStyle = "rgba(238,246,255,0.55)";
  for (let i = 0; i <= 4; i += 1) {
    const time = ((maxTime / 4) * i).toFixed(2);
    const x = padding.left + (plotWidth / 4) * i;
    context.fillText(time, x - 10, padding.top + plotHeight + 20);

    const value = (maxValue - (range / 4) * i).toFixed(2);
    const y = padding.top + (plotHeight / 4) * i + 4;
    context.fillText(value, 6, y);
  }
}

function hexToRgba(hex, alpha) {
  const value = hex.replace("#", "");
  const red = parseInt(value.slice(0, 2), 16);
  const green = parseInt(value.slice(2, 4), 16);
  const blue = parseInt(value.slice(4, 6), 16);
  return `rgba(${red}, ${green}, ${blue}, ${alpha})`;
}

function roundRect(context, x, y, width, height, radius) {
  context.beginPath();
  context.moveTo(x + radius, y);
  context.arcTo(x + width, y, x + width, y + height, radius);
  context.arcTo(x + width, y + height, x, y + height, radius);
  context.arcTo(x, y + height, x, y, radius);
  context.arcTo(x, y, x + width, y, radius);
  context.closePath();
}

function handleCdChange(value) {
  clearExternalCd();
  setManualCd(value);
}

async function handleModelUpload(event) {
  const file = event.target.files?.[0];
  if (!file) {
    return;
  }

  dom.fileStatus.textContent = "Parsing geometry...";

  try {
    const model = await parseModelFile(file);
    modelState.name = file.name;
    modelState.bounds = model.bounds;
    modelState.topProfile = model.topProfile;
    modelState.edgeSegments = model.edgeSegments;
    modelState.estimatedFrontalArea = clamp(model.estimatedFrontalArea || DEFAULT_AREA, 0.0001, 0.01);
    modelState.loaded = true;

    dom.modelName.textContent = file.name;
    dom.modelDimensions.textContent = `${model.bounds.length.toFixed(3)} x ${model.bounds.width.toFixed(3)} x ${model.bounds.height.toFixed(3)} m`;
    dom.modelArea.textContent = `${modelState.estimatedFrontalArea.toFixed(4)} m²`;
    dom.fileStatus.textContent = `Loaded ${file.name} with ${model.vertexCount.toLocaleString()} vertices`;
    render();
  } catch (error) {
    dom.fileStatus.textContent = `Upload failed: ${error.message}`;
  }
}

async function parseModelFile(file) {
  const extension = file.name.split(".").pop().toLowerCase();

  if (extension === "stl") {
    const buffer = await file.arrayBuffer();
    return buildRenderableModel(parseSTLVertices(buffer));
  }

  if (extension === "obj") {
    const text = await file.text();
    return buildRenderableModel(parseOBJData(text));
  }

  if (extension === "ply") {
    const text = await file.text();
    return buildRenderableModel(parsePLYData(text));
  }

  throw new Error("Unsupported format. Use STL, OBJ, or PLY.");
}

function parseSTLVertices(buffer) {
  const ascii = tryDecodeText(buffer);
  if (ascii.trim().startsWith("solid") && ascii.includes("facet normal")) {
    return parseAsciiSTL(ascii);
  }

  return parseBinarySTL(buffer);
}

function tryDecodeText(buffer) {
  try {
    return new TextDecoder().decode(buffer);
  } catch (_error) {
    return "";
  }
}

function parseAsciiSTL(text) {
  const vertices = [];
  const faces = [];
  const regex = /vertex\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)/g;
  let match;

  while ((match = regex.exec(text))) {
    vertices.push([Number(match[1]), Number(match[2]), Number(match[3])]);
  }

  for (let i = 0; i + 2 < vertices.length; i += 3) {
    faces.push([i, i + 1, i + 2]);
  }

  if (!vertices.length) {
    throw new Error("No vertices found in STL file");
  }

  return { vertices, faces };
}

function parseBinarySTL(buffer) {
  const view = new DataView(buffer);
  if (view.byteLength < 84) {
    throw new Error("STL file is too small");
  }

  const faceCount = view.getUint32(80, true);
  const vertices = [];
  const faces = [];
  let offset = 84;

  for (let faceIndex = 0; faceIndex < faceCount; faceIndex += 1) {
    offset += 12;
    const face = [];
    for (let vertexIndex = 0; vertexIndex < 3; vertexIndex += 1) {
      const x = view.getFloat32(offset, true);
      const y = view.getFloat32(offset + 4, true);
      const z = view.getFloat32(offset + 8, true);
      vertices.push([x, y, z]);
      face.push(vertices.length - 1);
      offset += 12;
    }
    faces.push(face);
    offset += 2;
  }

  if (!vertices.length) {
    throw new Error("No triangles found in STL file");
  }

  return { vertices, faces };
}

function parseOBJData(text) {
  const vertices = [];
  const faces = [];
  const lines = text.split(/\r?\n/);

  lines.forEach((line) => {
    const trimmed = line.trim();
    if (!trimmed || trimmed.startsWith("#")) {
      return;
    }

    if (trimmed.startsWith("v ")) {
      const parts = trimmed.split(/\s+/);
      vertices.push([Number(parts[1]), Number(parts[2]), Number(parts[3])]);
      return;
    }

    if (trimmed.startsWith("f ")) {
      const parts = trimmed.split(/\s+/).slice(1);
      const indices = parts
        .map((part) => Number(part.split("/")[0]))
        .filter((value) => Number.isFinite(value))
        .map((index) => (index < 0 ? vertices.length + index : index - 1));

      for (let i = 1; i + 1 < indices.length; i += 1) {
        faces.push([indices[0], indices[i], indices[i + 1]]);
      }
    }
  });

  if (!vertices.length) {
    throw new Error("No vertices found in OBJ file");
  }

  return { vertices, faces };
}

function parsePLYData(text) {
  const lines = text.split(/\r?\n/);
  if (lines[0]?.trim() !== "ply") {
    throw new Error("Only ASCII PLY files are supported here");
  }

  let vertexCount = 0;
  let faceCount = 0;
  let headerEndIndex = -1;

  for (let i = 0; i < lines.length; i += 1) {
    const line = lines[i].trim();
    if (line.startsWith("format") && !line.includes("ascii")) {
      throw new Error("Only ASCII PLY files are supported here");
    }
    if (line.startsWith("element vertex")) {
      vertexCount = Number(line.split(/\s+/)[2]);
    }
    if (line.startsWith("element face")) {
      faceCount = Number(line.split(/\s+/)[2]);
    }
    if (line === "end_header") {
      headerEndIndex = i;
      break;
    }
  }

  if (headerEndIndex === -1) {
    throw new Error("PLY header is incomplete");
  }

  const vertices = [];
  const faces = [];
  let cursor = headerEndIndex + 1;

  for (let i = 0; i < vertexCount; i += 1, cursor += 1) {
    const parts = lines[cursor].trim().split(/\s+/);
    vertices.push([Number(parts[0]), Number(parts[1]), Number(parts[2])]);
  }

  for (let i = 0; i < faceCount; i += 1, cursor += 1) {
    const parts = lines[cursor].trim().split(/\s+/).map(Number);
    const count = parts[0];
    const indices = parts.slice(1, count + 1);
    for (let j = 1; j + 1 < indices.length; j += 1) {
      faces.push([indices[0], indices[j], indices[j + 1]]);
    }
  }

  if (!vertices.length) {
    throw new Error("No vertices found in PLY file");
  }

  return { vertices, faces };
}

function buildRenderableModel(data) {
  const vertices = data.vertices.filter((vertex) => vertex.every((value) => Number.isFinite(value)));
  if (!vertices.length) {
    throw new Error("The uploaded model has no valid coordinates");
  }

  const bounds3D = getBounds3D(vertices);
  const axisOrder = ["x", "y", "z"].sort((a, b) => bounds3D.span[b] - bounds3D.span[a]);
  const lengthAxis = axisOrder[0];
  const widthAxis = axisOrder[1];
  const heightAxis = axisOrder[2];

  const projectedPoints = vertices.map((vertex) => ({
    x: normalizeAxis(vertex[axisToIndex(lengthAxis)], bounds3D.min[lengthAxis], bounds3D.max[lengthAxis]),
    y: normalizeAxis(vertex[axisToIndex(widthAxis)], bounds3D.min[widthAxis], bounds3D.max[widthAxis]),
  }));

  const topProfile = computeConvexHull(projectedPoints);
  const edgeSegments = buildEdgeSegments(vertices, data.faces, bounds3D, lengthAxis, widthAxis);
  const estimatedFrontalArea = bounds3D.span[widthAxis] * bounds3D.span[heightAxis];

  return {
    vertexCount: vertices.length,
    bounds: {
      length: bounds3D.span[lengthAxis],
      width: bounds3D.span[widthAxis],
      height: bounds3D.span[heightAxis],
    },
    estimatedFrontalArea,
    topProfile,
    edgeSegments,
  };
}

function axisToIndex(axisName) {
  if (axisName === "x") return 0;
  if (axisName === "y") return 1;
  return 2;
}

function getBounds3D(vertices) {
  const min = { x: Infinity, y: Infinity, z: Infinity };
  const max = { x: -Infinity, y: -Infinity, z: -Infinity };

  vertices.forEach((vertex) => {
    min.x = Math.min(min.x, vertex[0]);
    min.y = Math.min(min.y, vertex[1]);
    min.z = Math.min(min.z, vertex[2]);
    max.x = Math.max(max.x, vertex[0]);
    max.y = Math.max(max.y, vertex[1]);
    max.z = Math.max(max.z, vertex[2]);
  });

  return {
    min,
    max,
    span: {
      x: Math.max(max.x - min.x, 1e-9),
      y: Math.max(max.y - min.y, 1e-9),
      z: Math.max(max.z - min.z, 1e-9),
    },
  };
}

function normalizeAxis(value, min, max) {
  const midpoint = (min + max) * 0.5;
  const range = Math.max(max - min, 1e-9);
  return (value - midpoint) / range;
}

function buildEdgeSegments(vertices, faces, bounds3D, lengthAxis, widthAxis) {
  const edgeKeys = new Set();
  const segments = [];

  if (!faces.length) {
    return [];
  }

  for (let i = 0; i < faces.length; i += 1) {
    const face = faces[i];
    addEdge(face[0], face[1], vertices, segments, edgeKeys, bounds3D, lengthAxis, widthAxis);
    addEdge(face[1], face[2], vertices, segments, edgeKeys, bounds3D, lengthAxis, widthAxis);
    addEdge(face[2], face[0], vertices, segments, edgeKeys, bounds3D, lengthAxis, widthAxis);
    if (segments.length >= MAX_EDGE_COUNT) {
      break;
    }
  }

  return segments;
}

function addEdge(aIndex, bIndex, vertices, segments, edgeKeys, bounds3D, lengthAxis, widthAxis) {
  if (aIndex === bIndex) {
    return;
  }

  const key = aIndex < bIndex ? `${aIndex}-${bIndex}` : `${bIndex}-${aIndex}`;
  if (edgeKeys.has(key)) {
    return;
  }

  edgeKeys.add(key);
  const a = vertices[aIndex];
  const b = vertices[bIndex];
  segments.push({
    a: {
      x: normalizeAxis(a[axisToIndex(lengthAxis)], bounds3D.min[lengthAxis], bounds3D.max[lengthAxis]),
      y: normalizeAxis(a[axisToIndex(widthAxis)], bounds3D.min[widthAxis], bounds3D.max[widthAxis]),
    },
    b: {
      x: normalizeAxis(b[axisToIndex(lengthAxis)], bounds3D.min[lengthAxis], bounds3D.max[lengthAxis]),
      y: normalizeAxis(b[axisToIndex(widthAxis)], bounds3D.min[widthAxis], bounds3D.max[widthAxis]),
    },
  });
}

function computeConvexHull(points) {
  if (points.length <= 3) {
    return points.slice();
  }

  const sorted = points
    .slice()
    .sort((left, right) => (left.x === right.x ? left.y - right.y : left.x - right.x));

  const lower = [];
  sorted.forEach((point) => {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], point) <= 0) {
      lower.pop();
    }
    lower.push(point);
  });

  const upper = [];
  for (let i = sorted.length - 1; i >= 0; i -= 1) {
    const point = sorted[i];
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], point) <= 0) {
      upper.pop();
    }
    upper.push(point);
  }

  lower.pop();
  upper.pop();
  return lower.concat(upper);
}

function cross(origin, a, b) {
  return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

function createDefaultModel() {
  const topProfile = [
    { x: -0.48, y: 0.0 },
    { x: -0.3, y: -0.3 },
    { x: 0.18, y: -0.42 },
    { x: 0.46, y: -0.12 },
    { x: 0.5, y: 0.0 },
    { x: 0.46, y: 0.12 },
    { x: 0.18, y: 0.42 },
    { x: -0.3, y: 0.3 },
  ];

  const edgeSegments = [
    { a: { x: -0.26, y: -0.18 }, b: { x: 0.18, y: -0.28 } },
    { a: { x: -0.26, y: 0.18 }, b: { x: 0.18, y: 0.28 } },
    { a: { x: 0.02, y: -0.38 }, b: { x: 0.02, y: 0.38 } },
  ];

  return {
    name: "Default concept car",
    bounds: { length: 0.22, width: 0.05, height: 0.03 },
    topProfile,
    edgeSegments,
  };
}

dom.cdSlider.addEventListener("input", (event) => {
  handleCdChange(event.target.value);
});

dom.cdInput.addEventListener("input", (event) => {
  handleCdChange(event.target.value);
});

dom.useManualCd.addEventListener("click", () => {
  clearExternalCd();
  setManualCd(dom.cdInput.value);
});

dom.areaInput.addEventListener("input", (event) => {
  setReferenceArea(event.target.value);
});

dom.useModelArea.addEventListener("click", () => {
  setReferenceArea(modelState.estimatedFrontalArea);
});

dom.carFileInput.addEventListener("change", handleModelUpload);
dom.launchButton.addEventListener("click", launchSimulation);
dom.resetButton.addEventListener("click", resetSimulation);

window.addEventListener("resize", render);

syncCdInputs(dragState.manualCd);
setReferenceArea(DEFAULT_AREA);
updateCdSourceLabel();
dom.modelName.textContent = modelState.name;
dom.modelDimensions.textContent = `${modelState.bounds.length.toFixed(3)} x ${modelState.bounds.width.toFixed(3)} x ${modelState.bounds.height.toFixed(3)} m`;
dom.modelArea.textContent = `${DEFAULT_AREA.toFixed(4)} m²`;
updateMetrics();
render();
requestAnimationFrame(animate);
