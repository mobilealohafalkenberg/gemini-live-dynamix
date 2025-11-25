/**
 * ALOHA Robot Control App
 * Uses Gemini ER Robotics API via ER Bridge for robot control
 */

import { useState, useCallback } from "react";
import "./App.scss";
import { ALOHAControl } from "./components/aloha-control/ALOHAControl";
import ControlTray, { ConnectionInfo } from "./components/control-tray/ControlTray";
import { DualCameraView } from "./components/camera-feed/CameraFeed";

function App() {
  // Connection state lifted to App level
  const [robotConnected, setRobotConnected] = useState(false);
  const [connectionInfo, setConnectionInfo] = useState<ConnectionInfo | undefined>(undefined);

  // Handle connection changes from ControlTray
  const handleConnectionChange = useCallback((connected: boolean, info?: ConnectionInfo) => {
    setRobotConnected(connected);
    setConnectionInfo(info);
  }, []);

  return (
    <div className="App">
      <div className="streaming-console">
        <main style={{ width: '100%' }}>
          <h1 id="title" style={{ color: 'white', textAlign: 'center', margin: '20px 0' }}>
            ALOHA Robot Control
          </h1>

          <div className="main-app-area">
            {/* Main robot control component */}
            <ALOHAControl
              robotConnected={robotConnected}
              connectionInfo={connectionInfo}
            />

            {/* Robot camera feeds */}
            <DualCameraView enabled={robotConnected} />
          </div>

          {/* Control tray with connection controls */}
          <ControlTray onConnectionChange={handleConnectionChange} />
        </main>
      </div>
    </div>
  );
}

export default App;
