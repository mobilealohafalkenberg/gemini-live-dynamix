/**
 * Control Tray - Robot connection controls only
 * Connects to ER Bridge via WebSocket for robot control
 */

import { useEffect, useState, useRef } from "react";
import { useBridgeWebSocket } from "../../hooks/useBridgeWebSocket";
import "./control-tray.scss";

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

  // Track if we've received initial connection info
  const initializedRef = useRef(false);

  // Connection timeout ref
  const connectingTimeoutRef = useRef<NodeJS.Timeout>();

  // WebSocket connection to bridge
  const {
    connectionState,
    sendRobotConnect,
    sendRobotDisconnect,
  } = useBridgeWebSocket({
    onConnectionInfo: (info) => {
      // Initial connection info from bridge when WebSocket connects
      console.log('[ControlTray] Connection info received:', info);

      if (info.robot_connected) {
        setRobotConnected(true);
        onConnectionChange(true, {
          connected_arms: info.connected_arms,
          arm_count: info.connected_arms.length,
          cameras: info.cameras,
          gemini_api_ready: true,
        });
      } else {
        setRobotConnected(false);
        // Still pass connection info even if robot not connected
        onConnectionChange(false, {
          connected_arms: info.connected_arms,
          arm_count: info.connected_arms.length,
          cameras: info.cameras,
          gemini_api_ready: true,
        });
      }
      initializedRef.current = true;
    },
    onRobotStatus: (status) => {
      // Robot status update (after connect/disconnect)
      console.log('[ControlTray] Robot status update:', status);
      setRobotConnected(status.connected);
      setRobotConnecting(false);

      // Clear any pending timeout
      if (connectingTimeoutRef.current) {
        clearTimeout(connectingTimeoutRef.current);
        connectingTimeoutRef.current = undefined;
      }

      if (status.connected) {
        onConnectionChange(true, {
          connected_arms: status.connected_arms,
          arm_count: status.connected_arms.length,
          cameras: status.cameras || [],
          gemini_api_ready: true,
        });
      } else {
        onConnectionChange(false);
      }
    },
    onError: (err) => {
      console.error('[ControlTray] Bridge error:', err.code, err.message);
      setRobotConnecting(false);

      // Clear any pending timeout
      if (connectingTimeoutRef.current) {
        clearTimeout(connectingTimeoutRef.current);
        connectingTimeoutRef.current = undefined;
      }

      alert(`Bridge error: ${err.message}`);
    },
  });

  // Bridge status derived from WebSocket connection state
  const bridgeStatus = connectionState === 'connected' ? 'online' :
                       connectionState === 'connecting' ? 'unknown' : 'offline';

  // Handle robot connect via WebSocket
  const handleRobotConnect = () => {
    setRobotConnecting(true);
    sendRobotConnect();

    // Timeout fallback in case we don't get a response
    connectingTimeoutRef.current = setTimeout(() => {
      setRobotConnecting(false);
      console.warn('[ControlTray] Robot connect timeout');
    }, 10000);
  };

  // Handle robot disconnect via WebSocket
  const handleRobotDisconnect = () => {
    setRobotConnecting(true);
    sendRobotDisconnect();

    // Timeout fallback
    connectingTimeoutRef.current = setTimeout(() => {
      setRobotConnecting(false);
      console.warn('[ControlTray] Robot disconnect timeout');
    }, 10000);
  };

  // Cleanup timeout on unmount
  useEffect(() => {
    return () => {
      if (connectingTimeoutRef.current) {
        clearTimeout(connectingTimeoutRef.current);
      }
    };
  }, []);

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
