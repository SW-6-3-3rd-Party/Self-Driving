import { useEffect, useRef, useState } from "react";
import "./App.css";

const DEFAULT_COMMAND = {
  drive_command: "STOP",
  steering_command: "STRAIGHT",
  target_speed: 0.3,
};

const FEATURE_KEYS = [
  ["lkas_active", "LKAS"],
  ["lca_active", "LCA"],
  ["acc_active", "ACC"],
  ["aeb_active", "AEB"],
];

const steeringAngle = (command) => {
  if (command === "LEFT") return 0.25;
  if (command === "RIGHT") return -0.25;
  return 0.0;
};

const nearestFrontDistance = (frontAeb) => {
  if (!frontAeb) return null;
  const candidates = [
    frontAeb.front_tof_m,
    frontAeb.front_filtered_tof_m,
    frontAeb.front_obstacle_distance_m,
    frontAeb.front_left_diag_distance_m,
    frontAeb.front_right_diag_distance_m,
  ]
    .map(Number)
    .filter((value) => Number.isFinite(value) && value > 0);
  return candidates.length > 0 ? Math.min(...candidates) : null;
};

function App() {
  const [pcBackendUrl, setPcBackendUrl] = useState("http://127.0.0.1:8080");
  const [manualCommand, setManualCommand] = useState(DEFAULT_COMMAND);
  const [runtimeStatus, setRuntimeStatus] = useState(null);
  const [featureStatus, setFeatureStatus] = useState(null);
  const [featureFlags, setFeatureFlags] = useState({
    lkas_active: false,
    lca_active: false,
    acc_active: false,
    aeb_active: false,
  });
  const [otaStatus, setOtaStatus] = useState(null);
  const [packages, setPackages] = useState(null);
  const [versions, setVersions] = useState(null);
  const [otaTarget, setOtaTarget] = useState("HPVC");
  const [otaTargetVersion, setOtaTargetVersion] = useState("2.0.0");
  const [otaFilename, setOtaFilename] = useState("hpvc_2.0.0.zip");
  const [log, setLog] = useState([]);

  const commandRef = useRef(DEFAULT_COMMAND);
  const pressedKeysRef = useRef(new Set());
  const featureFlagsRef = useRef(featureFlags);
  const runtimeStatusRef = useRef(runtimeStatus);

  const addLog = (message) => {
    const time = new Date().toLocaleTimeString();
    setLog((prev) => [`[${time}] ${message}`, ...prev].slice(0, 45));
  };

  const requestPcBackendJson = async (path, options = {}) => {
    const response = await fetch(`${pcBackendUrl}${path}`, {
      headers: {
        "Content-Type": "application/json",
        ...(options.headers || {}),
      },
      ...options,
    });
    const data = await response.json();
    if (!response.ok) {
      throw new Error(data.reason || data.error || data.message || "request failed");
    }
    return data;
  };

  const normalizeFeatureFlags = (source) => ({
    lkas_active: Boolean(source?.lkas_active),
    lca_active: Boolean(source?.lca_active),
    acc_active: Boolean(source?.acc_active),
    aeb_active: Boolean(source?.aeb_active),
  });

  const requestedFeatureFlags = (source) => {
    if (source?.requested) {
      return normalizeFeatureFlags(source.requested);
    }
    if (
      source &&
      [
        "requested_lkas_enabled",
        "requested_lca_enabled",
        "requested_acc_enabled",
        "requested_aeb_enabled",
      ].some((key) => key in source)
    ) {
      return {
        lkas_active: Boolean(source.requested_lkas_enabled),
        lca_active: Boolean(source.requested_lca_enabled),
        acc_active: Boolean(source.requested_acc_enabled),
        aeb_active: Boolean(source.requested_aeb_enabled),
      };
    }
    return null;
  };

  const syncRuntimeState = (data) => {
    runtimeStatusRef.current = data;
    setRuntimeStatus(data);
    const flags = requestedFeatureFlags(data) || data?.feature_flags || data?.features;
    if (flags) {
      const normalizedFlags = normalizeFeatureFlags(flags);
      featureFlagsRef.current = normalizedFlags;
      setFeatureFlags(normalizedFlags);
    }
    if (data && !data.unavailable && !data.result) {
      const nextCommand = {
        drive_command: data.drive_command || "STOP",
        steering_command: data.steering_command || "STRAIGHT",
        target_speed: Number(data.target_speed ?? commandRef.current.target_speed),
      };
      commandRef.current = nextCommand;
      setManualCommand(nextCommand);
    }
  };

  const isLcaControlRequested = () =>
    Boolean(featureFlagsRef.current.lca_active || runtimeStatusRef.current?.requested_lca_enabled);

  const refreshRuntimeStatus = async ({ quiet = false } = {}) => {
    try {
      const data = await requestPcBackendJson("/api/vehicle/runtime/status");
      syncRuntimeState(data);
      if (!quiet) addLog("HPVC runtime status updated");
      return data;
    } catch (error) {
      const unavailable = {
        unavailable: true,
        reason: error.message,
      };
      runtimeStatusRef.current = unavailable;
      setRuntimeStatus(unavailable);
      if (!quiet) addLog(`HPVC runtime unavailable: ${error.message}`);
      return null;
    }
  };

  const refreshFeatureStatus = async ({ quiet = false } = {}) => {
    try {
      const data = await requestPcBackendJson("/api/vehicle/runtime/features");
      setFeatureStatus(data);
      const flags = requestedFeatureFlags(data) || data.feature_flags || data.features;
      if (flags) {
        const normalizedFlags = normalizeFeatureFlags(flags);
        featureFlagsRef.current = normalizedFlags;
        setFeatureFlags(normalizedFlags);
      }
      if (!quiet) addLog("Feature status updated");
      return data;
    } catch (error) {
      const unavailable = {
        unavailable: true,
        reason: error.message,
      };
      setFeatureStatus(unavailable);
      if (!quiet) addLog(`Feature status unavailable: ${error.message}`);
      return null;
    }
  };

  const sendRuntimeControl = async (patch, { quiet = false } = {}) => {
    const previousCommand = commandRef.current;
    const nextCommand = {
      ...previousCommand,
      ...patch,
    };
    commandRef.current = nextCommand;
    setManualCommand(nextCommand);

    try {
      const data = await requestPcBackendJson("/api/vehicle/runtime/manual", {
        method: "POST",
        body: JSON.stringify(nextCommand),
      });
      syncRuntimeState(data);
      if (!quiet) {
        addLog(
          `Runtime command: drive=${data.drive_command}, steer=${data.steering_command}, speed=${Number(
            data.target_speed ?? nextCommand.target_speed
          ).toFixed(2)}`
        );
      }
      return data;
    } catch (error) {
      commandRef.current = previousCommand;
      setManualCommand(previousCommand);
      addLog(`Runtime command failed: ${error.message}`);
      return null;
    }
  };

  const sendTurnSignal = async (turnSignal, { quiet = false } = {}) => {
    try {
      const data = await requestPcBackendJson("/api/vehicle/runtime/turn-signal", {
        method: "POST",
        body: JSON.stringify({ turn_signal: turnSignal }),
      });
      syncRuntimeState(data);
      if (!quiet) addLog(`LCA turn signal: ${data.turn_signal || turnSignal}`);
      return data;
    } catch (error) {
      addLog(`LCA turn signal failed: ${error.message}`);
      return null;
    }
  };

  const stopRuntime = async ({ quiet = false } = {}) => {
    await sendRuntimeControl(
      {
        drive_command: "STOP",
        steering_command: "STRAIGHT",
      },
      { quiet: true }
    );
    await sendTurnSignal("OFF", { quiet: true });
    if (!quiet) addLog("STOP sent to HPVC runtime");
  };

  const resetRuntime = async () => {
    pressedKeysRef.current.clear();
    commandRef.current = DEFAULT_COMMAND;
    setManualCommand(DEFAULT_COMMAND);
    try {
      const data = await requestPcBackendJson("/api/vehicle/runtime/reset", {
        method: "POST",
        body: JSON.stringify({}),
      });
      syncRuntimeState(data);
      addLog("HPVC runtime reset");
    } catch (error) {
      addLog(`Runtime reset failed: ${error.message}`);
    }
  };

  const updateSpeed = (value) => {
    const targetSpeed = Number(value);
    commandRef.current = {
      ...commandRef.current,
      target_speed: targetSpeed,
    };
    setManualCommand(commandRef.current);
    sendRuntimeControl({ target_speed: targetSpeed }, { quiet: true });
  };

  const setDrive = (driveCommand) => {
    sendRuntimeControl({ drive_command: driveCommand });
  };

  const releaseDrive = () => {
    sendRuntimeControl({ drive_command: "STOP" }, { quiet: true });
  };

  const setSteering = (steeringCommand) => {
    if (isLcaControlRequested()) {
      sendTurnSignal(steeringCommand);
      return;
    }
    sendRuntimeControl({ steering_command: steeringCommand });
  };

  const releaseSteering = () => {
    if (isLcaControlRequested()) {
      return;
    }
    sendRuntimeControl({ steering_command: "STRAIGHT" }, { quiet: true });
  };

  const centerSteeringOrClearLca = () => {
    if (isLcaControlRequested()) {
      sendTurnSignal("OFF");
      return;
    }
    sendRuntimeControl({ steering_command: "STRAIGHT" });
  };

  const applyKeyboardCommand = () => {
    const keys = pressedKeysRef.current;
    const drive = keys.has("w") ? "FORWARD" : keys.has("s") ? "REVERSE" : "STOP";
    if (isLcaControlRequested()) {
      if (drive !== commandRef.current.drive_command) {
        sendRuntimeControl(
          {
            drive_command: drive,
            steering_command: "STRAIGHT",
          },
          { quiet: true }
        );
      }
      const turnSignal = keys.has("a") ? "LEFT" : keys.has("d") ? "RIGHT" : null;
      if (turnSignal && turnSignal !== (runtimeStatusRef.current?.turn_signal || "OFF")) {
        sendTurnSignal(turnSignal, { quiet: true });
      }
      return;
    }

    const steering = keys.has("a") ? "LEFT" : keys.has("d") ? "RIGHT" : "STRAIGHT";

    if (
      drive === commandRef.current.drive_command &&
      steering === commandRef.current.steering_command
    ) {
      return;
    }

    sendRuntimeControl(
      {
        drive_command: drive,
        steering_command: steering,
      },
      { quiet: true }
    );
  };

  const updateFeatureFlag = async (key, value) => {
    const nextFlags = {
      ...featureFlags,
      [key]: value,
    };
    featureFlagsRef.current = nextFlags;
    setFeatureFlags(nextFlags);

    try {
      const data = await requestPcBackendJson("/api/vehicle/runtime/features", {
        method: "POST",
        body: JSON.stringify({
          [key]: value,
        }),
      });
      syncRuntimeState(data);
      setFeatureStatus(data);
      if (key === "lca_active" && !value) {
        await sendTurnSignal("OFF", { quiet: true });
      }
      addLog(`${key} ${value ? "armed" : "disabled"}`);
    } catch (error) {
      addLog(`Feature update failed: ${error.message}`);
      refreshFeatureStatus({ quiet: true });
    }
  };

  const getPackages = async () => {
    try {
      const data = await requestPcBackendJson("/api/ota/packages");
      setPackages(data);
      addLog(`OTA packages loaded: ${data.packages?.length || 0}`);
    } catch (error) {
      addLog(`OTA package list failed: ${error.message}`);
    }
  };

  const getOtaStatus = async ({ quiet = false } = {}) => {
    try {
      const data = await requestPcBackendJson("/api/ota/status");
      setOtaStatus(data);
      if (!quiet) addLog("PC OTA status updated");
      return data;
    } catch (error) {
      if (!quiet) addLog(`PC OTA status failed: ${error.message}`);
      return null;
    }
  };

  const getVersions = async () => {
    try {
      const data = await requestPcBackendJson("/api/vehicle/runtime/version");
      setVersions(data);
      addLog("HPVC version updated");
    } catch (error) {
      addLog(`HPVC version failed: ${error.message}`);
    }
  };

  const checkPcBackend = async () => {
    try {
      const data = await requestPcBackendJson("/api/ota/health");
      addLog(`PC backend OK: mqtt=${data.mqtt_connected}, runtime=${data.hpvc_runtime_base_url}`);
    } catch (error) {
      addLog(`PC backend check failed: ${error.message}`);
    }
  };

  const startOta = async () => {
    try {
      const data = await requestPcBackendJson("/api/ota/start", {
        method: "POST",
        body: JSON.stringify({
          target: otaTarget,
          target_version: otaTargetVersion,
          filename: otaFilename,
        }),
      });
      addLog(`OTA job published: ${data.job.job_id}`);
      setOtaStatus({
        latest_job: data.job,
        latest_status: {
          state: "JOB_PUBLISHED",
          progress: 0,
          running: true,
        },
      });
    } catch (error) {
      addLog(`OTA start failed: ${error.message}`);
    }
  };

  useEffect(() => {
    const handleKeyDown = (event) => {
      if (event.target.matches("input, select, textarea, button")) return;

      const key = event.key.toLowerCase();
      if (key === " ") {
        event.preventDefault();
        pressedKeysRef.current.clear();
        stopRuntime();
        return;
      }

      if (!["w", "a", "s", "d"].includes(key) || event.repeat) return;
      event.preventDefault();
      pressedKeysRef.current.add(key);
      applyKeyboardCommand();
    };

    const handleKeyUp = (event) => {
      const key = event.key.toLowerCase();
      if (!["w", "a", "s", "d"].includes(key)) return;
      event.preventDefault();
      pressedKeysRef.current.delete(key);
      applyKeyboardCommand();
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
    // Keyboard listeners intentionally read the latest command from refs.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pcBackendUrl]);

  useEffect(() => {
    const initialTimer = window.setTimeout(() => {
      refreshRuntimeStatus({ quiet: true });
      refreshFeatureStatus({ quiet: true });
      getOtaStatus({ quiet: true });
    }, 0);

    const runtimeTimer = window.setInterval(() => {
      refreshRuntimeStatus({ quiet: true });
    }, 500);

    const featureTimer = window.setInterval(() => {
      refreshFeatureStatus({ quiet: true });
    }, 1500);

    const otaTimer = window.setInterval(() => {
      getOtaStatus({ quiet: true });
    }, 2000);

    return () => {
      window.clearTimeout(initialTimer);
      window.clearInterval(runtimeTimer);
      window.clearInterval(featureTimer);
      window.clearInterval(otaTimer);
    };
    // Polling intentionally restarts only when the selected PC backend changes.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pcBackendUrl]);

  const runtimeOnline = Boolean(runtimeStatus && !runtimeStatus.unavailable && !runtimeStatus.result);
  const aebActive = Boolean(runtimeStatus?.aeb_active || runtimeStatus?.current_mode === "AEB");
  const aebArmed = Boolean(featureFlags.aeb_active || runtimeStatus?.requested_aeb_enabled);
  const lcaRequested = Boolean(featureFlags.lca_active || runtimeStatus?.requested_lca_enabled);
  const currentDrive = runtimeStatus?.drive_command || manualCommand.drive_command;
  const currentSteering = runtimeStatus?.steering_command || manualCommand.steering_command;
  const currentTurnSignal = runtimeStatus?.turn_signal || "OFF";
  const currentSpeed = Number(runtimeStatus?.target_speed ?? manualCommand.target_speed);
  const frontDistance = nearestFrontDistance(runtimeStatus?.front_aeb);

  return (
    <div className="app">
      <header className="topbar">
        <div>
          <h1>HPVC PC HMI</h1>
          <p>Runtime API control, ADAS status, AEB guard, and OTA management</p>
        </div>
        <div className="connection-line">
          <span className={`badge ${runtimeOnline ? "ok" : "bad"}`}>
            Runtime {runtimeOnline ? "ONLINE" : "OFFLINE"}
          </span>
          <span className={`badge ${aebActive ? "bad" : aebArmed ? "ok" : "warn"}`}>
            AEB {aebActive ? "BRAKING" : aebArmed ? "ARMED" : "GUARD"}
          </span>
        </div>
      </header>

      <section className="panel connection-panel">
        <div className="field">
          <label>PC backend</label>
          <input value={pcBackendUrl} onChange={(event) => setPcBackendUrl(event.target.value)} />
        </div>
        <div className="actions">
          <button onClick={checkPcBackend}>Check backend</button>
          <button onClick={() => refreshRuntimeStatus()}>Runtime status</button>
          <button onClick={() => refreshFeatureStatus()}>Feature status</button>
          <button onClick={resetRuntime}>Reset runtime</button>
        </div>
      </section>

      <main className="layout">
        <section className="panel drive-panel">
          <div className="section-title">
            <h2>Vehicle Control</h2>
            <span className={`status-pill ${aebActive ? "danger" : runtimeOnline ? "ok" : "warn"}`}>
              {aebActive ? "AEB BRAKE" : runtimeOnline ? "READY" : "NO RUNTIME"}
            </span>
          </div>

          <div className="command-readout">
            <div>
              <span>Drive</span>
              <strong>{currentDrive}</strong>
            </div>
            <div>
              <span>Steer</span>
              <strong>{currentSteering}</strong>
            </div>
            <div>
              <span>Angle</span>
              <strong>{steeringAngle(currentSteering).toFixed(3)} rad</strong>
            </div>
            <div>
              <span>LCA signal</span>
              <strong>{currentTurnSignal}</strong>
            </div>
          </div>

          <div className="speed-control">
            <label htmlFor="target-speed">Target speed</label>
            <input
              id="target-speed"
              type="range"
              min="0.05"
              max="1.0"
              step="0.05"
              value={manualCommand.target_speed}
              onChange={(event) => updateSpeed(event.target.value)}
            />
            <output>{Number(manualCommand.target_speed).toFixed(2)} m/s</output>
          </div>

          <div className="control-pad" onContextMenu={(event) => event.preventDefault()}>
            <button
              className="pad-button forward"
              onPointerDown={() => setDrive("FORWARD")}
              onPointerUp={releaseDrive}
              onPointerLeave={releaseDrive}
              onPointerCancel={releaseDrive}
            >
              FORWARD
            </button>
            <div className="pad-row">
              <button
                className="pad-button steer"
                onPointerDown={() => setSteering("LEFT")}
                onPointerUp={releaseSteering}
                onPointerLeave={releaseSteering}
                onPointerCancel={releaseSteering}
              >
                {lcaRequested ? "LCA LEFT" : "LEFT"}
              </button>
              <button className="pad-button stop" onClick={() => stopRuntime()}>
                STOP
              </button>
              <button
                className="pad-button steer"
                onPointerDown={() => setSteering("RIGHT")}
                onPointerUp={releaseSteering}
                onPointerLeave={releaseSteering}
                onPointerCancel={releaseSteering}
              >
                {lcaRequested ? "LCA RIGHT" : "RIGHT"}
              </button>
            </div>
            <button
              className="pad-button reverse"
              onPointerDown={() => setDrive("REVERSE")}
              onPointerUp={releaseDrive}
              onPointerLeave={releaseDrive}
              onPointerCancel={releaseDrive}
            >
              REVERSE
            </button>
          </div>

          <div className="actions critical-actions">
            <button className="danger-button" onClick={() => stopRuntime()}>
              STOP NOW
            </button>
            <button onClick={centerSteeringOrClearLca}>
              {lcaRequested ? "Clear LCA signal" : "Center steering"}
            </button>
          </div>

          <p className="hint">
            Keyboard: W/S drive, A/D steer or LCA signal, Space stop. Commands go through hpvc_runtime_api.py.
          </p>
        </section>

        <section className="panel status-panel">
          <div className="section-title">
            <h2>HPVC Runtime Link</h2>
            <button onClick={() => refreshRuntimeStatus()}>Refresh</button>
          </div>
          <dl className="compact-list">
            <dt>Current mode</dt>
            <dd>{runtimeStatus?.current_mode || "-"}</dd>
            <dt>AEB reason</dt>
            <dd>{runtimeStatus?.aeb_reason || runtimeStatus?.reason || "-"}</dd>
            <dt>Front distance</dt>
            <dd>{typeof frontDistance === "number" ? `${frontDistance.toFixed(3)} m` : "-"}</dd>
            <dt>Rear speed</dt>
            <dd>
              {typeof runtimeStatus?.rear_status?.rear_vehicle_speed_mps === "number"
                ? `${runtimeStatus.rear_status.rear_vehicle_speed_mps.toFixed(3)} m/s`
                : "-"}
            </dd>
            <dt>Target speed</dt>
            <dd>{currentSpeed.toFixed(2)} m/s</dd>
          </dl>
          <pre>{runtimeStatus ? JSON.stringify(runtimeStatus, null, 2) : "-"}</pre>
        </section>

        <section className="panel">
          <div className="section-title">
            <h2>ADAS Runtime</h2>
            <button onClick={() => refreshFeatureStatus()}>Refresh</button>
          </div>

          <div className="toggle-grid">
            {FEATURE_KEYS.map(([key, label]) => (
              <label className="toggle-row" key={key}>
                <span>{label}</span>
                <input
                  type="checkbox"
                  checked={featureFlags[key]}
                  onChange={(event) => updateFeatureFlag(key, event.target.checked)}
                />
              </label>
            ))}
          </div>

          <pre>{featureStatus ? JSON.stringify(featureStatus, null, 2) : "-"}</pre>
        </section>

        <section className="panel">
          <div className="section-title">
            <h2>AEB Guard</h2>
            <button onClick={() => updateFeatureFlag("aeb_active", !aebArmed)}>
              {aebArmed ? "Disable AEB request" : "Arm AEB request"}
            </button>
          </div>
          <dl className="compact-list">
            <dt>Armed</dt>
            <dd>{aebArmed ? "YES" : "AUTO GUARD"}</dd>
            <dt>Braking</dt>
            <dd>{aebActive ? "YES" : "NO"}</dd>
            <dt>Trigger</dt>
            <dd>{"front ToF <= 0.18 m"}</dd>
            <dt>Sensor</dt>
            <dd>{runtimeStatus?.front_aeb?.front_packet_format || "-"}</dd>
          </dl>
        </section>

        <section className="panel ota-panel">
          <div className="section-title">
            <h2>MQTT OTA Manager</h2>
            <div className="actions">
              <button onClick={getPackages}>Packages</button>
              <button onClick={() => getOtaStatus()}>Status</button>
              <button onClick={getVersions}>HPVC version</button>
            </div>
          </div>

          <div className="ota-form">
            <div className="field">
              <label>Target</label>
              <select value={otaTarget} onChange={(event) => setOtaTarget(event.target.value)}>
                <option value="HPVC">HPVC</option>
                <option value="CENTER_RPI">CENTER_RPI</option>
                <option value="FRONT_ZONE">FRONT_ZONE</option>
                <option value="REAR_ZONE">REAR_ZONE</option>
              </select>
            </div>
            <div className="field">
              <label>Version</label>
              <input value={otaTargetVersion} onChange={(event) => setOtaTargetVersion(event.target.value)} />
            </div>
            <div className="field">
              <label>Package</label>
              <input value={otaFilename} onChange={(event) => setOtaFilename(event.target.value)} />
            </div>
            <button className="update-button" onClick={startOta}>Publish OTA job</button>
          </div>

          <div className="split-status">
            <pre>{otaStatus ? JSON.stringify(otaStatus, null, 2) : "-"}</pre>
            <pre>{packages ? JSON.stringify(packages, null, 2) : versions ? JSON.stringify(versions, null, 2) : "-"}</pre>
          </div>
        </section>

        <section className="panel log-panel">
          <div className="section-title">
            <h2>Event Log</h2>
            <button onClick={() => setLog([])}>Clear</button>
          </div>
          <pre>{log.length > 0 ? log.join("\n") : "-"}</pre>
        </section>
      </main>
    </div>
  );
}

export default App;
