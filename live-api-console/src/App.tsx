/**
 * ALOHA Robot Control App
 * Uses WebSocket connection to ER Bridge for real-time robot control
 */

import { useState, useCallback } from "react";
import "./App.scss";
import { ALOHAControl } from "./components/aloha-control/ALOHAControl";
import { DualCameraView } from "./components/camera-feed/CameraFeed";
import { CameraFrames, RobotStatus } from "./hooks/useBridgeWebSocket";

function App() {
  // Camera frames from WebSocket (shared between ALOHAControl and DualCameraView)
  const [cameraFrames, setCameraFrames] = useState<CameraFrames | null>(null);
  const [robotStatus, setRobotStatus] = useState<RobotStatus | null>(null);
  const [wsConnected, setWsConnected] = useState(false);

  // Handle camera frames from ALOHAControl WebSocket
  const handleCameraFrames = useCallback((frames: CameraFrames) => {
    setCameraFrames(frames);
    setWsConnected(true);
  }, []);

  // Handle robot status changes from WebSocket
  const handleRobotStatusChange = useCallback((status: RobotStatus) => {
    setRobotStatus(status);
    setWsConnected(true);
  }, []);

  const robotConnected = robotStatus?.connected ?? false;

  return (
    <div className="App">
      <div className="streaming-console">
        <main style={{ width: '100%' }}>
          <h1 id="title" style={{ color: 'white', textAlign: 'center', margin: '20px 0' }}>
            ALOHA Robot Control
          </h1>

          <div className="main-app-area">
            {/* Main robot control component with WebSocket */}
            <ALOHAControl
              onCameraFrames={handleCameraFrames}
              onRobotStatusChange={handleRobotStatusChange}
            />

            {/* Robot camera feeds (receives frames from WebSocket via ALOHAControl) */}
            <DualCameraView
              frames={cameraFrames}
              isConnected={wsConnected && robotConnected}
            />
          </div>

          {/* Status footer */}
          <div style={{
            textAlign: 'center',
            padding: '8px',
            color: '#6b7280',
            fontSize: 12
          }}>
            WebSocket: {wsConnected ? 'Connected' : 'Connecting...'} |
            Robot: {robotConnected ? 'Ready' : 'Not connected'} |
            Arms: {robotStatus?.connected_arms?.join(', ') || 'None'}
          </div>
        </main>
      </div>
    </div>
  );
}

export default App;
