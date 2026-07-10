import { BridgeSocket } from "./ws-client.js";

const qs = (id) => document.getElementById(id);

const ui = {
  conn: qs("conn"),
  x: qs("x"),
  y: qs("y"),
  front: qs("front"),
  backLeft: qs("backLeft"),
  backRight: qs("backRight"),
  drop: qs("drop"),
  telemetryJson: qs("telemetryJson"),
  hMode: qs("hMode"),
  hMission: qs("hMission"),
  hWaypoint: qs("hWaypoint"),
  hGeofence: qs("hGeofence"),
  hDockImpact: qs("hDockImpact"),
  hCrc: qs("hCrc"),
  hTxFail: qs("hTxFail"),
  hLastAck: qs("hLastAck"),
  missionId: qs("missionId"),
  landingMode: qs("landingMode"),
  waypoints: qs("waypoints"),
  geofenceEnabled: qs("geofenceEnabled"),
  geofenceRadius: qs("geofenceRadius"),
  geofenceLat: qs("geofenceLat"),
  geofenceLng: qs("geofenceLng"),
  applyGeofence: qs("applyGeofence"),
  tuneDockPwmPct: qs("tuneDockPwmPct"),
  tuneImpactG: qs("tuneImpactG"),
  tuneRthHaltM: qs("tuneRthHaltM"),
  tuneRthDecelM: qs("tuneRthDecelM"),
  applyTuning: qs("applyTuning"),
  uploadMission: qs("uploadMission"),
  startMission: qs("startMission"),
  pauseMission: qs("pauseMission"),
  abortMission: qs("abortMission"),
  rthMission: qs("rthMission"),
  missionStatus: qs("missionStatus")
};

const wsUrl = `ws://${location.host}/ws`;
const bridge = new BridgeSocket(wsUrl);
bridge.connect();

bridge.onState((up) => {
  ui.conn.textContent = up ? "Connected" : "Disconnected";
  ui.conn.classList.toggle("online", up);
  ui.conn.classList.toggle("offline", !up);
});

bridge.onFrame((frame) => {
  if (frame?.type === "telemetry.frame") {
    ui.telemetryJson.textContent = JSON.stringify(frame, null, 2);
    renderHealth(frame);
  }
});

function setBadge(el, ok, okText, badText) {
  el.textContent = ok ? okText : badText;
  el.classList.toggle("ok", ok);
  el.classList.toggle("bad", !ok);
}

function renderHealth(frame) {
  const boat = frame?.boat ?? {};
  const link = frame?.link ?? {};
  ui.hMode.textContent = boat.mode ?? "-";
  ui.hMission.textContent = boat.missionRunning ? "Running" : "Idle";

  const active = Number(boat.activeWaypoint ?? 0);
  const total = Number(boat.waypointCount ?? 0);
  ui.hWaypoint.textContent = `${active}/${total}`;

  setBadge(ui.hGeofence, Boolean(boat.geofenceOk), "OK", "OUT");
  setBadge(ui.hDockImpact, Boolean(boat.dockImpact), "IMPACT", "CLEAR");

  ui.hCrc.textContent = String(boat.crcInvalidRxCount ?? 0);
  ui.hTxFail.textContent = String(boat.radioTxFailCount ?? 0);

  const lastAckMs = Number(link.lastAckMs ?? -1);
  ui.hLastAck.textContent = lastAckMs >= 0 ? `${lastAckMs} ms` : "-";
}

function setMissionStatus(text) {
  ui.missionStatus.textContent = text;
}

function getLandingMode() {
  return ui.landingMode.value === "IN_BOAT" ? "IN_BOAT" : "ON_SHORE";
}

function sendMissionCommand(cmd) {
  const ok = bridge.send({
    type: "mission.command",
    ts: Date.now(),
    cmd,
    landingMode: getLandingMode()
  });
  setMissionStatus(ok ? `Sent mission command: ${cmd}` : "Failed to send mission command (disconnected)");
}

ui.applyGeofence.addEventListener("click", () => {
  const enabled = ui.geofenceEnabled.checked;
  const centerLat = Number(ui.geofenceLat.value);
  const centerLng = Number(ui.geofenceLng.value);
  const radiusM = Number(ui.geofenceRadius.value);

  if (!Number.isFinite(centerLat) || !Number.isFinite(centerLng) || !Number.isFinite(radiusM)) {
    setMissionStatus("Geofence values are invalid.");
    return;
  }

  const ok = bridge.send({
    type: "config.geofence",
    ts: Date.now(),
    enabled,
    centerLat,
    centerLng,
    radiusM
  });

  setMissionStatus(
    ok
      ? `Geofence ${enabled ? "enabled" : "disabled"} (${radiusM.toFixed(1)}m).`
      : "Failed to send geofence config (disconnected)"
  );
});

ui.applyTuning.addEventListener("click", () => {
  const dockPwmPct = Number(ui.tuneDockPwmPct.value);
  const impactThresholdG = Number(ui.tuneImpactG.value);
  const rthHaltM = Number(ui.tuneRthHaltM.value);
  const rthDecelStartM = Number(ui.tuneRthDecelM.value);

  if (!Number.isFinite(dockPwmPct) || !Number.isFinite(impactThresholdG) ||
      !Number.isFinite(rthHaltM) || !Number.isFinite(rthDecelStartM)) {
    setMissionStatus("Tuning values are invalid.");
    return;
  }

  const ok = bridge.send({
    type: "config.tuning",
    ts: Date.now(),
    dockPwmPct,
    impactThresholdG,
    rthHaltM,
    rthDecelStartM
  });

  setMissionStatus(
    ok
      ? `Tuning applied (dock ${dockPwmPct.toFixed(1)}%, impact ${impactThresholdG.toFixed(2)}g).`
      : "Failed to send tuning config (disconnected)"
  );
});

ui.uploadMission.addEventListener("click", () => {
  let waypoints = [];
  try {
    const parsed = JSON.parse(ui.waypoints.value);
    if (!Array.isArray(parsed)) {
      setMissionStatus("Waypoints JSON must be an array.");
      return;
    }
    waypoints = parsed;
  } catch {
    setMissionStatus("Invalid JSON in waypoints input.");
    return;
  }

  const missionId = Number(ui.missionId.value);
  const ok = bridge.send({
    type: "mission.upload",
    ts: Date.now(),
    missionId: Number.isFinite(missionId) ? missionId : 1,
    landingMode: getLandingMode(),
    waypoints
  });
  setMissionStatus(ok ? `Mission uploaded with ${waypoints.length} waypoint(s).` : "Failed to upload mission (disconnected)");
});

ui.startMission.addEventListener("click", () => sendMissionCommand("START"));
ui.pauseMission.addEventListener("click", () => sendMissionCommand("PAUSE"));
ui.abortMission.addEventListener("click", () => sendMissionCommand("ABORT"));
ui.rthMission.addEventListener("click", () => sendMissionCommand("RTH"));

let dropPulse = false;
ui.drop.addEventListener("click", () => {
  dropPulse = true;
});

setInterval(() => {
  bridge.send({
    type: "control.manual",
    ts: Date.now(),
    x: Number(ui.x.value),
    y: Number(ui.y.value),
    headingLock: false,
    targetHeadingDeg: 0,
    lights: {
      front: ui.front.checked,
      backLeft: ui.backLeft.checked,
      backRight: ui.backRight.checked
    },
    servo: {
      drop: dropPulse
    }
  });

  dropPulse = false;
}, 50);
