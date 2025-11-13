/**
 * Original Control Tray with full voice and video controls
 */

import { useEffect, useState } from "react";
import { useLiveAPIContext } from "../../contexts/LiveAPIContext";
import { AudioRecorder } from "../../lib/audio-recorder";
import "./control-tray.scss";

type ControlTrayProps = {
  videoRef: React.RefObject<HTMLVideoElement>;
  onVideoStreamChange?: (stream: MediaStream | null) => void;
};

function ControlTray({ videoRef, onVideoStreamChange }: ControlTrayProps) {
  const { client, connected, connect, disconnect, volume } = useLiveAPIContext();
  const [audioRecorder] = useState(() => new AudioRecorder());
  const [videoStream, setVideoStream] = useState<MediaStream | null>(null);
  const [muted, setMuted] = useState(false);
  const [inVolume, setInVolume] = useState(0);
  const [connecting, setConnecting] = useState(false);
  const [cameraMode, setCameraMode] = useState<'both' | 'gripper' | 'top' | 'merged' | 'none'>('merged');

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (audioRecorder) {
        audioRecorder.stop();
      }
    };
  }, [audioRecorder]);

  // Mute/unmute handling
  const handleMuteToggle = () => {
    setMuted(!muted);
  };

  // Connection handling
  const handleConnect = async () => {
    setConnecting(true);
    try {
      // Start video stream
      const stream = await navigator.mediaDevices.getUserMedia({
        video: true,
        audio: false
      });
      setVideoStream(stream);
      if (videoRef.current) {
        videoRef.current.srcObject = stream;
      }
      onVideoStreamChange?.(stream);

      // Connect to Gemini
      await connect();

      // Send video stream if available
      if (stream && client) {
        await client.sendMedia({ video: stream });
      }
    } catch (error) {
      console.error("Connection failed:", error);
    } finally {
      setConnecting(false);
    }
  };

  const handleDisconnect = async () => {
    await disconnect();
    if (videoStream) {
      videoStream.getTracks().forEach(track => track.stop());
      setVideoStream(null);
    }
    if (videoRef.current) {
      videoRef.current.srcObject = null;
    }
    onVideoStreamChange?.(null);
    if (audioRecorder) {
      audioRecorder.stop();
    }
  };

  // Helper function to merge two camera frames horizontally with labels
  const mergeFrames = async (gripperB64: string, topB64: string): Promise<string> => {
    return new Promise((resolve) => {
      const canvas = document.createElement('canvas');
      const ctx = canvas.getContext('2d');
      
      const img1 = new Image();
      const img2 = new Image();
      
      let loadedCount = 0;
      
      const onLoad = () => {
        loadedCount++;
        if (loadedCount === 2 && ctx) {
          // Set canvas size (both images side by side)
          canvas.width = 640 * 2; // 1280 total width
          canvas.height = 480; // Keep original height
          
          // Fill background
          ctx.fillStyle = '#000000';
          ctx.fillRect(0, 0, canvas.width, canvas.height);
          
          // Draw LEFT GRIPPER camera on the left
          ctx.drawImage(img1, 0, 0, 640, 480);
          
          // Draw TOP camera on the right
          ctx.drawImage(img2, 640, 0, 640, 480);
          
          // Add labels with background for visibility
          ctx.font = 'bold 24px Arial';
          ctx.textAlign = 'center';
          
          // Left label
          ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
          ctx.fillRect(10, 10, 200, 40);
          ctx.fillStyle = '#00FF00';
          ctx.fillText('LEFT GRIPPER', 110, 38);
          
          // Right label
          ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
          ctx.fillRect(650, 10, 200, 40);
          ctx.fillStyle = '#00FFFF';
          ctx.fillText('TOP VIEW', 750, 38);
          
          // Convert to base64 JPEG
          const base64 = canvas.toDataURL('image/jpeg', 0.85);
          resolve(base64.split(',')[1]);
        }
      };
      
      img1.onload = onLoad;
      img2.onload = onLoad;
      img1.src = `data:image/jpeg;base64,${gripperB64}`;
      img2.src = `data:image/jpeg;base64,${topB64}`;
    });
  };

  // Send robot camera frames periodically based on selected mode
  useEffect(() => {
    if (!connected || !client || cameraMode === 'none') return;

    let timeoutId: number;
    const ROBOT_ENDPOINT = process.env.REACT_APP_ROBOT_ENDPOINT || 'http://localhost:8081';
    
    const sendRobotCameraFrames = async () => {
      try {
        const framesToSend = [];
        
        // Fetch frames based on camera mode
        if (cameraMode === 'both') {
          const [gripperRes, topRes] = await Promise.all([
            fetch(`${ROBOT_ENDPOINT}/camera/gripper_cam/frame`),
            fetch(`${ROBOT_ENDPOINT}/camera/top_cam/frame`)
          ]);
          
          if (gripperRes.ok && topRes.ok) {
            const gripperData = await gripperRes.json();
            const topData = await topRes.json();
            if (gripperData.success) framesToSend.push({ mimeType: "image/jpeg", data: gripperData.frame });
            if (topData.success) framesToSend.push({ mimeType: "image/jpeg", data: topData.frame });
          }
        } else if (cameraMode === 'merged') {
          // Fetch both cameras and merge into single frame
          const [gripperRes, topRes] = await Promise.all([
            fetch(`${ROBOT_ENDPOINT}/camera/gripper_cam/frame`),
            fetch(`${ROBOT_ENDPOINT}/camera/top_cam/frame`)
          ]);
          
          if (gripperRes.ok && topRes.ok) {
            const gripperData = await gripperRes.json();
            const topData = await topRes.json();
            
            if (gripperData.success && topData.success) {
              const mergedFrame = await mergeFrames(gripperData.frame, topData.frame);
              framesToSend.push({ mimeType: "image/jpeg", data: mergedFrame });
              console.log('🎞️ Created merged frame with labels');
            }
          }
        } else if (cameraMode === 'gripper') {
          const res = await fetch(`${ROBOT_ENDPOINT}/camera/gripper_cam/frame`);
          if (res.ok) {
            const data = await res.json();
            if (data.success) framesToSend.push({ mimeType: "image/jpeg", data: data.frame });
          }
        } else if (cameraMode === 'top') {
          const res = await fetch(`${ROBOT_ENDPOINT}/camera/top_cam/frame`);
          if (res.ok) {
            const data = await res.json();
            if (data.success) framesToSend.push({ mimeType: "image/jpeg", data: data.frame });
          }
        }
        
        // Send frames if we have any
        if (framesToSend.length > 0) {
          client.sendRealtimeInput(framesToSend);
          console.log(`📸 Sent ${cameraMode} camera(s) to Gemini`);
        }
      } catch (e) {
        console.error('Failed to send robot camera frames:', e);
      }
      
      if (connected) {
        // Send frames every 500ms (2 FPS) for better spatial reasoning
        timeoutId = window.setTimeout(sendRobotCameraFrames, 500);
      }
    };

    sendRobotCameraFrames();

    return () => {
      if (timeoutId) {
        clearTimeout(timeoutId);
      }
    };
  }, [connected, client, cameraMode]);

  // Handle audio recording
  useEffect(() => {
    const onData = (base64: string) => {
      if (connected && client) {
        client.sendRealtimeInput([{
          mimeType: "audio/pcm;rate=16000",
          data: base64,
        }]);
      }
    };

    if (connected && !muted && audioRecorder) {
      audioRecorder.on("data", onData).on("volume", setInVolume).start();
    } else if (audioRecorder) {
      audioRecorder.stop();
    }

    return () => {
      if (audioRecorder) {
        audioRecorder.off("data", onData).off("volume", setInVolume);
      }
    };
  }, [connected, client, muted, audioRecorder]);

  return (
    <div className="control-tray">
      <div className="control-tray-container">
        {/* Connection Controls */}
        <div className="control-group">
          {!connected ? (
            <button
              className="control-button connect-button"
              onClick={handleConnect}
              disabled={connecting}
            >
              {connecting ? "Connecting..." : "🔌 Connect"}
            </button>
          ) : (
            <button
              className="control-button disconnect-button"
              onClick={handleDisconnect}
            >
              ⏹ Disconnect
            </button>
          )}
        </div>

        {/* Camera Selection */}
        {connected && (
          <div className="control-group">
            <label style={{ fontSize: '12px', color: '#888', marginBottom: '4px' }}>
              Camera Feed:
            </label>
            <select 
              value={cameraMode} 
              onChange={(e) => setCameraMode(e.target.value as any)}
              style={{
                padding: '8px',
                borderRadius: '4px',
                border: '1px solid #333',
                background: '#2a2a2a',
                color: 'white',
                fontSize: '14px',
                marginBottom: '8px',
                width: '100%'
              }}
            >
              <option value="merged">Merged View (Recommended)</option>
              <option value="both">Both Cameras (Separate)</option>
              <option value="gripper">Gripper Only</option>
              <option value="top">Top Only</option>
              <option value="none">No Camera</option>
            </select>
          </div>
        )}

        {/* Voice Controls */}
        {connected && (
          <div className="control-group">
            <button
              className={`control-button ${muted ? 'muted' : ''}`}
              onClick={handleMuteToggle}
            >
              {muted ? '🔇 Unmute' : '🎤 Mute'}
            </button>
            
            {/* Volume indicators */}
            <div className="volume-indicators">
              <div className="volume-bar">
                <span className="volume-label">In</span>
                <div className="volume-meter">
                  <div 
                    className="volume-level"
                    style={{ width: `${inVolume * 100}%` }}
                  />
                </div>
              </div>
              <div className="volume-bar">
                <span className="volume-label">Out</span>
                <div className="volume-meter">
                  <div 
                    className="volume-level"
                    style={{ width: `${volume * 100}%` }}
                  />
                </div>
              </div>
            </div>
          </div>
        )}

        {/* Status Indicators */}
        <div className="status-indicators">
          <span className="status-item">
            Bridge: {connected ? '🟢' : '🔴'}
          </span>
          <span className="status-item">
            Video: {videoStream ? '📹' : '❌'}
          </span>
          <span className="status-item">
            Audio: {!muted && connected ? '🎤' : '🔇'}
          </span>
        </div>
      </div>
    </div>
  );
}

export default ControlTray;