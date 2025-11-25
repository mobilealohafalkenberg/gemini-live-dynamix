/**
 * Control Tray - Robot connection controls only
 * Connects to ER Bridge for robot and Gemini ER API access
 */

import { useEffect, useState } from "react";
import "./control-tray.scss";

// ER Bridge endpoint configuration
const ER_BRIDGE_ENDPOINT = process.env.REACT_APP_ER_BRIDGE_ENDPOINT || 'http://localhost:8082';

// Connection info type (matches ALOHAControl)
export interface ConnectionInfo {
  connected_arms: string[];
  arm_count: number;
  cameras: string[];
  gemini_api_ready: boolean;
}

interface ControlTrayProps {
  onConnectionChange: (connected: boolean, info?: ConnectionInfo) => void;
}

function ControlTray({ onConnectionChange }: ControlTrayProps) {
  // Robot connection state
  const [robotConnected, setRobotConnected] = useState(false);
  const [robotConnecting, setRobotConnecting] = useState(false);
  const [bridgeStatus, setBridgeStatus] = useState<'unknown' | 'online' | 'offline'>('unknown');

  // Fetch additional status info after connection
  const fetchConnectionInfo = async (): Promise<ConnectionInfo | null> => {
    try {
      // Fetch robot status and camera info in parallel
      const [robotRes, cameraRes, statusRes] = await Promise.all([
        fetch(`${ER_BRIDGE_ENDPOINT}/robot/status`),
        fetch(`${ER_BRIDGE_ENDPOINT}/camera/info`),
        fetch(`${ER_BRIDGE_ENDPOINT}/status`)
      ]);

      const robotData = await robotRes.json();
      const cameraData = await cameraRes.json();
      const statusData = await statusRes.json();

      return {
        connected_arms: robotData.connected_arms || [],
        arm_count: robotData.arm_count || 0,
        cameras: cameraData.cameras || [],
        gemini_api_ready: statusData.api_key_set || false
      };
    } catch (error) {
      console.error('Failed to fetch connection info:', error);
      return null;
    }
  };

  // Robot connection handling
  const handleRobotConnect = async () => {
    setRobotConnecting(true);
    try {
      const response = await fetch(`${ER_BRIDGE_ENDPOINT}/robot/connect`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
      });
      const data = await response.json();

      if (data.success) {
        setRobotConnected(true);
        console.log('Robot connected:', data.message);

        // Fetch full connection info
        const connectionInfo = await fetchConnectionInfo();
        onConnectionChange(true, connectionInfo || undefined);
      } else {
        console.error('Robot connection failed:', data.error);
        alert(`Robot connection failed: ${data.error}`);
        onConnectionChange(false);
      }
    } catch (error) {
      console.error('Robot connection error:', error);
      alert(`Robot connection error: ${error}`);
      onConnectionChange(false);
    } finally {
      setRobotConnecting(false);
    }
  };

  const handleRobotDisconnect = async () => {
    setRobotConnecting(true);
    try {
      const response = await fetch(`${ER_BRIDGE_ENDPOINT}/robot/disconnect`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
      });
      const data = await response.json();

      if (data.success) {
        setRobotConnected(false);
        console.log('Robot disconnected:', data.message);
        onConnectionChange(false);
      } else {
        console.error('Robot disconnection failed:', data.error);
        alert(`Robot disconnection failed: ${data.error}`);
      }
    } catch (error) {
      console.error('Robot disconnection error:', error);
      alert(`Robot disconnection error: ${error}`);
    } finally {
      setRobotConnecting(false);
    }
  };

  // Check bridge status and robot connection on mount
  useEffect(() => {
    const checkStatus = async () => {
      try {
        const response = await fetch(`${ER_BRIDGE_ENDPOINT}/status`);
        const data = await response.json();
        setBridgeStatus('online');

        // Check if robot is already connected
        if (data.robot_connected) {
          setRobotConnected(true);
          const connectionInfo = await fetchConnectionInfo();
          onConnectionChange(true, connectionInfo || undefined);
        }
      } catch (error) {
        console.warn('ER Bridge not reachable:', error);
        setBridgeStatus('offline');
        setRobotConnected(false);
        onConnectionChange(false);
      }
    };

    checkStatus();

    // Poll status every 5 seconds
    const interval = setInterval(checkStatus, 5000);
    return () => clearInterval(interval);
  }, [onConnectionChange]);

  return (
    <div className="control-tray">
      <div className="control-tray-container">
        {/* Robot Connection Controls */}
        <div className="control-group">
          {!robotConnected ? (
            <button
              className="control-button connect-button"
              onClick={handleRobotConnect}
              disabled={robotConnecting || bridgeStatus === 'offline'}
              style={{ backgroundColor: bridgeStatus === 'offline' ? '#4b5563' : '#2a5a2a' }}
            >
              {robotConnecting ? "Connecting..." : bridgeStatus === 'offline' ? "Bridge Offline" : "Connect Robot"}
            </button>
          ) : (
            <button
              className="control-button disconnect-button"
              onClick={handleRobotDisconnect}
              disabled={robotConnecting}
              style={{ backgroundColor: '#5a2a2a' }}
            >
              {robotConnecting ? "Disconnecting..." : "Disconnect Robot"}
            </button>
          )}
        </div>

        {/* Status Indicators */}
        <div className="status-indicators">
          <span className="status-item">
            Bridge: {bridgeStatus === 'online' ? 'Online' : bridgeStatus === 'offline' ? 'Offline' : 'Checking...'}
          </span>
          <span className="status-item">
            Robot: {robotConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
      </div>
    </div>
  );
}

export default ControlTray;
