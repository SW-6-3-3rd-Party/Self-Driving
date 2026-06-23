import { useEffect, useState } from "react";
import "./App.css";

function App() {
  const [hpvcIp, setHpvcIp] = useState("192.168.201.17");
  const [vehicleStatus, setVehicleStatus] = useState(null);
  const [otaStatus, setOtaStatus] = useState(null);
  const [versions, setVersions] = useState(null);
  const [log, setLog] = useState([]);

  const baseUrl = `http://${hpvcIp}:8000`;

  const addLog = (message) => {
    const time = new Date().toLocaleTimeString();
    setLog((prev) => [`[${time}] ${message}`, ...prev].slice(0, 20));
  };

  const requestJson = async (path, options = {}) => {
    const response = await fetch(`${baseUrl}${path}`, {
      headers: {
        "Content-Type": "application/json",
        ...(options.headers || {}),
      },
      ...options,
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.reason || data.error || "Request failed");
    }

    return data;
  };

  const getVehicleStatus = async () => {
    try {
      const data = await requestJson("/vehicle/status");
      setVehicleStatus(data);
      addLog("차량 상태 조회 성공");
    } catch (error) {
      addLog(`차량 상태 조회 실패: ${error.message}`);
    }
  };

  const getOtaStatus = async () => {
    try {
      const data = await requestJson("/ota/status");
      setOtaStatus(data);
      addLog(`OTA 상태 조회: ${data.state}, ${data.progress}%`);
    } catch (error) {
      addLog(`OTA 상태 조회 실패: ${error.message}`);
    }
  };

  const getVersions = async () => {
    try {
      const data = await requestJson("/version");
      setVersions(data);
      addLog("버전 조회 성공");
    } catch (error) {
      addLog(`버전 조회 실패: ${error.message}`);
    }
  };

  const sendManualControl = async (driveCommand, steeringCommand, targetSpeed) => {
    try {
      const data = await requestJson("/control/manual", {
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
      const data = await requestJson("/control/turn-signal", {
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
      const data = await requestJson("/ota/start", {
        method: "POST",
      });

      addLog(data.message || "OTA 시작 요청 성공");
      pollOtaStatus();
    } catch (error) {
      addLog(`OTA 시작 실패: ${error.message}`);
    }
  };

  const pollOtaStatus = () => {
    const timer = setInterval(async () => {
      try {
        const data = await requestJson("/ota/status");
        setOtaStatus(data);

        if (!data.running) {
          clearInterval(timer);
          addLog(`OTA 종료: ${data.state}, ${data.progress}%`);
          getVersions();
        }
      } catch (error) {
        clearInterval(timer);
        addLog(`OTA polling 실패: ${error.message}`);
      }
    }, 1000);
  };

  useEffect(() => {
    const handleKeyDown = (event) => {
      if (event.target.tagName === "INPUT") return;
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
        <p>Manual Control · Turn Signal · OTA Manager</p>
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
          <h2>OTA Manager</h2>

          <div className="button-row">
            <button onClick={getVersions}>버전 확인</button>
            <button onClick={getOtaStatus}>OTA 상태</button>
            <button className="update" onClick={startOta}>업데이트 시작</button>
          </div>

          <div className="status-box">
            <h3>OTA Status</h3>
            <pre>{otaStatus ? JSON.stringify(otaStatus, null, 2) : "-"}</pre>
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