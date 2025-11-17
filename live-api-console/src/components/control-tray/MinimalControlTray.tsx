/**
 * Minimal Control Tray - just Connect/Disconnect and basic status
 */

import { useState } from "react";
import { useLiveAPIContext } from "../../contexts/LiveAPIContext";
import { useWebcam } from "../../hooks/use-webcam";
import "./control-tray.scss";

type MinimalControlTrayProps = {
  videoRef: React.RefObject<HTMLVideoElement>;
  onVideoStreamChange?: (stream: MediaStream | null) => void;
};

function MinimalControlTray({ videoRef, onVideoStreamChange }: MinimalControlTrayProps) {
  const { client, connected, connect, disconnect } = useLiveAPIContext();
  const [connecting, setConnecting] = useState(false);
  const webcam = useWebcam();

  const handleConnect = async () => {
    setConnecting(true);
    try {
      // Start webcam for video input
      const stream = await webcam.start();
      if (videoRef.current && stream) {
        videoRef.current.srcObject = stream;
        onVideoStreamChange?.(stream);
      }
      
      // Connect to Gemini
      await connect();
      
      // Send video stream if available
      if (stream) {
        const videoTrack = stream.getVideoTracks()[0];
        if (videoTrack) {
          // Send video to Gemini
          await client.sendMedia({ video: stream });
        }
      }
    } catch (error) {
      console.error("Connection failed:", error);
    } finally {
      setConnecting(false);
    }
  };

  const handleDisconnect = async () => {
    await disconnect();
    webcam.stop();
    if (videoRef.current) {
      videoRef.current.srcObject = null;
    }
    onVideoStreamChange?.(null);
  };

  return (
    <div className="control-tray" style={{ 
      position: 'fixed', 
      bottom: 0, 
      left: 0, 
      right: 0,
      background: '#1f2937',
      padding: '16px',
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
      gap: '20px',
      borderTop: '1px solid #374151',
      zIndex: 1000
    }}>
      {/* Status indicator on left */}
      <div style={{ 
        padding: '8px 16px',
        background: '#374151',
        borderRadius: '6px',
        color: '#9ca3af',
        fontSize: '14px'
      }}>
        Bridge: 🟢 | Video: {webcam.stream ? '📹' : '❌'}
      </div>

      {/* Connection button in center */}
      <div>
        {!connected ? (
          <button 
            onClick={handleConnect}
            disabled={connecting}
            style={{
              padding: '12px 24px',
              background: '#10b981',
              color: 'white',
              border: 'none',
              borderRadius: '8px',
              fontSize: '16px',
              fontWeight: 'bold',
              cursor: connecting ? 'wait' : 'pointer',
              opacity: connecting ? 0.5 : 1
            }}
          >
            {connecting ? 'Connecting...' : '🔌 Connect to Gemini'}
          </button>
        ) : (
          <>
            <span style={{ color: '#10b981', fontWeight: 'bold', marginRight: '20px' }}>
              ✅ Connected
            </span>
            <button 
              onClick={handleDisconnect}
              style={{
                padding: '12px 24px',
                background: '#ef4444',
                color: 'white',
                border: 'none',
                borderRadius: '8px',
                fontSize: '16px',
                fontWeight: 'bold',
                cursor: 'pointer'
              }}
            >
              🔌 Disconnect
            </button>
          </>
        )}
      </div>
      
      {/* Empty space on right for balance */}
      <div style={{ width: '150px' }}></div>
    </div>
  );
}

export default MinimalControlTray;