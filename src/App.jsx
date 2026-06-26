import { useEffect, useState } from "react";
import "./App.css";

function App() {
  const [hpvcIp, setHpvcIp] = useState("192.168.137.50");
  const [pcBackendUrl, setPcBackendUrl] = useState("http://192.168.137.1:8080");

  const [vehicleStatus, setVehicleStatus] = useState(null);
  const [otaStatus, setOtaStatus] = useState(null);
  const [versions, setVersions] = useState(null);
  const [packages, setPackages] = useState(null);

  const [otaTarget, setOtaTarget] = useState("HPVC");
  const [otaTargetVersion, setOtaTargetVersion] = useState("2.0.0");
  const [otaFilename, setOtaFilename] = useState("hpvc_2.0.0.zip");

  const [log, setLog] = useState([]);

  const hpvcBaseUrl = `http://${hpvcIp}:8000`;

  const addLog = (message) => {
    const time = new Date().toLocaleTimeString();
    setLog((prev) => [`[${time}] ${message}`, ...prev].slice(0, 30));
  };

  const requestHpvcJson = async (path, options = {}) => {
    const response = await fetch(`${hpvcBaseUrl}${path}`, {
      headers: {
        "Content-Type": "application/json",
        ...(options.headers || {}),
      },
      ...options,
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.reason || data.error || data.message || "HPVC request failed");
    }

    return data;
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
      throw new Error(data.reason || data.error || data.message || "PC Backend request failed");
    }

    return data;
  };

  const getVehicleStatus = async () => {
    try {
      const data = await requestHpvcJson("/vehicle/status");
      setVehicleStatus(data);
      addLog("차량 상태 조회 성공");
    } catch (error) {
      addLog(`차량 상태 조회 실패: ${error.message}`);
    }
  };

  const getHpvcOtaStatus = async () => {
    try {
      const data = await requestHpvcJson("/ota/status");
      addLog(`HPVC OTA 상태: ${data.state}, ${data.progress}%`);
      return data;
    } catch (error) {
      addLog(`HPVC OTA 상태 조회 실패: ${error.message}`);
      return null;
    }
  };

  const getPcOtaStatus = async () => {
    try {
      const data = await requestPcBackendJson("/api/ota/status");
      setOtaStatus(data);

      const latest = data.latest_status;
      if (latest) {
        addLog(`PC OTA 상태 조회: ${latest.state}, ${latest.progress}%`);
      } else {
        addLog("PC OTA 상태 조회 성공");
      }

      return data;
    } catch (error) {
      addLog(`PC OTA 상태 조회 실패: ${error.message}`);
      return null;
    }
  };

  const getOtaStatus = async () => {
    await getPcOtaStatus();
  };

  const getVersions = async () => {
    try {
      const data = await requestHpvcJson("/version");
      setVersions(data);
      addLog("HPVC 버전 조회 성공");
    } catch (error) {
      addLog(`버전 조회 실패: ${error.message}`);
    }
  };

  const getPackages = async () => {
    try {
      const data = await requestPcBackendJson("/api/ota/packages");
      setPackages(data);
      addLog(`OTA 패키지 목록 조회 성공: ${data.packages?.length || 0}개`);
    } catch (error) {
      addLog(`OTA 패키지 목록 조회 실패: ${error.message}`);
    }
  };

  const checkPcBackend = async () => {
    try {
      const data = await requestPcBackendJson("/api/ota/health");
      addLog(
        `PC OTA Backend OK: mqtt_connected=${data.mqtt_connected}, artifact=${data.artifact_base_url}`
      );
    } catch (error) {
      addLog(`PC OTA Backend 확인 실패: ${error.message}`);
    }
  };

  const sendManualControl = async (driveCommand, steeringCommand, targetSpeed) => {
    try {
      const data = await requestHpvcJson("/control/manual", {
        method: "POST",
        body: JSON.stringify({
          drive_command: driveCommand,
          steering_command: steeringCommand,
          target_speed: targetSpeed,
        }),
      });

      addLog(
        `수동조작 전송: drive=${data.drive_command}, steering=${data.steering_command}, speed=${data.target_speed}`
      );

      await getVehicleStatus();
    } catch (error) {
      addLog(`수동조작 실패: ${error.message}`);
    }
  };

  const sendTurnSignal = async (turnSignal) => {
    try {
      const data = await requestHpvcJson("/control/turn-signal", {
        method: "POST",
        body: JSON.stringify({
          turn_signal: turnSignal,
        }),
      });

      addLog(`방향지시등 전송: ${data.turn_signal}, turn_request=${data.turn_request}`);
      await getVehicleStatus();
    } catch (error) {
      addLog(`방향지시등 실패: ${error.message}`);
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

      addLog(`OTA Job publish 성공: ${data.job.job_id}`);
      setOtaStatus({
        latest_job: data.job,
        latest_status: {
          state: "JOB_PUBLISHED",
          progress: 0,
          running: true,
        },
      });

      pollOtaStatus();
    } catch (error) {
      addLog(`OTA 시작 실패: ${error.message}`);
    }
  };

  const pollOtaStatus = () => {
    const timer = setInterval(async () => {
      const data = await getPcOtaStatus();

      if (!data || !data.latest_status) {
        return;
      }

      const latest = data.latest_status;

      if (!latest.running) {
        clearInterval(timer);

        addLog(`OTA 종료: ${latest.state}, ${latest.progress}%`);

        await getVersions();
        await getHpvcOtaStatus();
      }
    }, 1000);
  };

  useEffect(() => {
    const handleKeyDown = (event) => {
      if (event.target.tagName === "INPUT" || event.target.tagName === "SELECT") return;
      if (event.repeat) return;

      switch (event.key) {
        case "ArrowUp":
          sendManualControl("FORWARD", "STRAIGHT", 0.3);
          break;
        case "ArrowDown":
          sendManualControl("REVERSE", "STRAIGHT", 0.2);
          break;
        case "ArrowLeft":
          sendManualControl("FORWARD", "LEFT", 0.3);
          break;
        case "ArrowRight":
          sendManualControl("FORWARD", "RIGHT", 0.3);
          break;
        case " ":
          event.preventDefault();
          sendManualControl("STOP", "STRAIGHT", 0.0);
          break;
        default:
          break;
      }
    };

    window.addEventListener("keydown", handleKeyDown);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, [hpvcIp]);

  return (
    <div className="app">
      <header className="header">
        <h1>HPVC PC HMI</h1>
        <p>Manual Control · Turn Signal · MQTT OTA Manager</p>
      </header>

      <section className="card connection-card">
        <h2>Connection</h2>

        <div className="input-row">
          <label>HPVC IP</label>
          <input
            value={hpvcIp}
            onChange={(e) => setHpvcIp(e.target.value)}
          />
        </div>

        <div className="input-row">
          <label>PC OTA Backend URL</label>
          <input
            value={pcBackendUrl}
            onChange={(e) => setPcBackendUrl(e.target.value)}
          />
        </div>

        <div className="button-row">
          <button onClick={checkPcBackend}>PC Backend 확인</button>
          <button onClick={getVehicleStatus}>차량 상태 조회</button>
          <button onClick={getVersions}>버전 조회</button>
        </div>
      </section>

      <main className="grid">
        <section className="card">
          <h2>Manual Control</h2>

          <div className="control-pad">
            <button onClick={() => sendManualControl("FORWARD", "STRAIGHT", 0.3)}>
              ↑ 전진
            </button>

            <div className="middle-row">
              <button onClick={() => sendManualControl("FORWARD", "LEFT", 0.3)}>
                ← 좌회전
              </button>
              <button
                className="stop"
                onClick={() => sendManualControl("STOP", "STRAIGHT", 0.0)}
              >
                정지
              </button>
              <button onClick={() => sendManualControl("FORWARD", "RIGHT", 0.3)}>
                우회전 →
              </button>
            </div>

            <button onClick={() => sendManualControl("REVERSE", "STRAIGHT", 0.2)}>
              ↓ 후진
            </button>
          </div>

          <p className="hint">키보드 방향키와 Space도 사용 가능</p>
        </section>

        <section className="card">
          <h2>Turn Signal</h2>

          <div className="button-row">
            <button onClick={() => sendTurnSignal("LEFT")}>좌측</button>
            <button onClick={() => sendTurnSignal("RIGHT")}>우측</button>
            <button onClick={() => sendTurnSignal("OFF")}>해제</button>
          </div>
        </section>

        <section className="card">
          <h2>MQTT OTA Manager</h2>

          <div className="input-row">
            <label>Target</label>
            <select
              value={otaTarget}
              onChange={(e) => setOtaTarget(e.target.value)}
            >
              <option value="HPVC">HPVC</option>
              <option value="CENTER_RPI">CENTER_RPI</option>
              <option value="FRONT_ZONE">FRONT_ZONE</option>
              <option value="REAR_ZONE">REAR_ZONE</option>
            </select>
          </div>

          <div className="input-row">
            <label>Target Version</label>
            <input
              value={otaTargetVersion}
              onChange={(e) => setOtaTargetVersion(e.target.value)}
            />
          </div>

          <div className="input-row">
            <label>Package Filename</label>
            <input
              value={otaFilename}
              onChange={(e) => setOtaFilename(e.target.value)}
            />
          </div>

          <div className="button-row">
            <button onClick={getPackages}>패키지 목록</button>
            <button onClick={getOtaStatus}>OTA 상태</button>
            <button className="update" onClick={startOta}>
              MQTT OTA 시작
            </button>
          </div>

          <div className="status-box">
            <h3>OTA Status</h3>
            <pre>{otaStatus ? JSON.stringify(otaStatus, null, 2) : "-"}</pre>
          </div>

          <div className="status-box">
            <h3>Packages</h3>
            <pre>{packages ? JSON.stringify(packages, null, 2) : "-"}</pre>
          </div>

          <div className="status-box">
            <h3>Versions</h3>
            <pre>{versions ? JSON.stringify(versions, null, 2) : "-"}</pre>
          </div>
        </section>

        <section className="card">
          <h2>Vehicle Status</h2>

          <button onClick={getVehicleStatus}>차량 상태 조회</button>

          <pre>{vehicleStatus ? JSON.stringify(vehicleStatus, null, 2) : "-"}</pre>
        </section>
      </main>

      <section className="card log-card">
        <h2>Log</h2>
        <pre>{log.length > 0 ? log.join("\n") : "-"}</pre>
      </section>
    </div>
  );
}

export default App;
