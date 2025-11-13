/**
 * Minimal ALOHA Gripper Control App
 * Stripped down to essentials - just gripper control via Gemini Live API
 */

import { useRef, useState } from "react";
import "./App.scss";
import { LiveAPIProvider } from "./contexts/LiveAPIContext";
import { ALOHAControl } from "./components/aloha-control/ALOHAControl";
import ControlTray from "./components/control-tray/ControlTray";
import { DualCameraView } from "./components/camera-feed/CameraFeed";
import { LiveClientOptions } from "./types";

const API_KEY = process.env.REACT_APP_GEMINI_API_KEY as string;
if (typeof API_KEY !== "string") {
  throw new Error("set REACT_APP_GEMINI_API_KEY in .env");
}

const apiOptions: LiveClientOptions = {
  apiKey: API_KEY,
};

function App() {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [videoStream, setVideoStream] = useState<MediaStream | null>(null);

  return (
    <div className="App">
      <LiveAPIProvider options={apiOptions}>
        <div className="streaming-console">
          <main style={{ width: '100%' }}>
            <div className="main-app-area">
              <h1 style={{ color: 'white', textAlign: 'center', margin: '20px 0' }}>
                🤖 ALOHA Gripper Control
              </h1>
              
              {/* Main gripper control component */}
              <ALOHAControl />
              
              {/* Robot camera feeds */}
              <DualCameraView enabled={true} />
              
              {/* Video stream (for webcam if needed) */}
              <video
                className="stream"
                ref={videoRef}
                autoPlay
                playsInline
                style={{ 
                  display: videoStream ? 'block' : 'none',
                  maxWidth: '400px',
                  margin: '20px auto',
                  borderRadius: '8px'
                }}
              />
            </div>

            {/* Control tray with voice and connection controls */}
            <ControlTray
              videoRef={videoRef}
              onVideoStreamChange={setVideoStream}
            />
          </main>
        </div>
      </LiveAPIProvider>
    </div>
  );
}

export default App;
