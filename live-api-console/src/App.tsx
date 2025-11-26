/**
 * ALOHA Robot Control App
 * Uses WebSocket connection to ER Bridge for real-time robot control
 */

import { useState, useCallback } from "react";
import "./App.scss";
import { ALOHAControl } from "./components/aloha-control/ALOHAControl";
import { RobotStatus } from "./hooks/useBridgeWebSocket";

function App() {
  const [robotStatus, setRobotStatus] = useState<RobotStatus | null>(null);
  const [wsConnected, setWsConnected] = useState(false);

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
            <ALOHAControl onRobotStatusChange={handleRobotStatusChange} />
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
