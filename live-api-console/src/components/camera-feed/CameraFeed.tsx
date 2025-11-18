import { useEffect, useState, useRef } from 'react';

const ROBOT_ENDPOINT = process.env.REACT_APP_ROBOT_ENDPOINT || 'http://localhost:8081';

interface CameraFeedProps {
  cameraName: string;
  title: string;
  width?: number;
  height?: number;
  refreshRate?: number;
}

export function CameraFeed({ 
  cameraName, 
  title, 
  width = 320, 
  height = 240,
  refreshRate = 100  // milliseconds between frame updates
}: CameraFeedProps) {
  const [imageUrl, setImageUrl] = useState<string>('');
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string>('');
  const intervalRef = useRef<NodeJS.Timeout>();
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    let mounted = true;

    const fetchFrame = async () => {
      try {
        const response = await fetch(`${ROBOT_ENDPOINT}/camera/${cameraName}/frame`);
        if (!response.ok) {
          throw new Error(`Failed to fetch frame: ${response.statusText}`);
        }
        
        const data = await response.json();
        if (data.success && data.frame && mounted) {
          // Convert base64 to image URL
          const imageUrl = `data:image/jpeg;base64,${data.frame}`;
          setImageUrl(imageUrl);
          setIsLoading(false);
          setError('');
          
          // Draw to canvas for Gemini
          if (canvasRef.current) {
            const img = new Image();
            img.onload = () => {
              const ctx = canvasRef.current?.getContext('2d');
              if (ctx && canvasRef.current) {
                ctx.drawImage(img, 0, 0, width, height);
              }
            };
            img.src = imageUrl;
          }
        }
      } catch (err: any) {
        if (mounted) {
          console.error(`Camera feed error (${cameraName}):`, err);
          setError(err.message);
          setIsLoading(false);
        }
      }
    };

    // Initial fetch
    fetchFrame();

    // Set up interval for continuous updates
    intervalRef.current = setInterval(fetchFrame, refreshRate);

    return () => {
      mounted = false;
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, [cameraName, refreshRate, width, height]);

  return (
    <div style={{ 
      display: 'inline-block', 
      margin: 8,
      background: '#1f2937',
      borderRadius: 8,
      padding: 8,
      minWidth: width,
    }}>
      <h4 style={{ margin: '0 0 8px 0', color: 'white', fontSize: 14 }}>
        📹 {title}
      </h4>
      
      {isLoading && !imageUrl && (
        <div style={{ 
          width, 
          height, 
          background: '#374151',
          borderRadius: 4,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          color: '#9ca3af'
        }}>
          Loading camera...
        </div>
      )}
      
      {error && (
        <div style={{ 
          width, 
          height, 
          background: '#374151',
          borderRadius: 4,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          color: '#ef4444',
          fontSize: 12,
          padding: 8,
          textAlign: 'center'
        }}>
          ❌ {error}
        </div>
      )}
      
      {imageUrl && !error && (
        <>
          <img
            src={imageUrl}
            alt={title}
            style={{
              width,
              height,
              borderRadius: 4,
              display: 'block',
              objectFit: 'cover'
            }}
          />
          {/* Hidden canvas for Gemini API */}
          <canvas
            ref={canvasRef}
            width={width}
            height={height}
            style={{ display: 'none' }}
            id={`canvas-${cameraName}`}
          />
        </>
      )}
      
      <div style={{ 
        marginTop: 4, 
        fontSize: 10, 
        color: '#6b7280',
        textAlign: 'center'
      }}>
        {cameraName}
      </div>
    </div>
  );
}

interface DualCameraViewProps {
  enabled?: boolean;
}

export function DualCameraView({ enabled = true }: DualCameraViewProps) {
  const [cameraInfo, setCameraInfo] = useState<any>(null);

  useEffect(() => {
    if (!enabled) return;

    // Fetch camera info
    fetch(`${ROBOT_ENDPOINT}/camera/info`)
      .then(res => res.json())
      .then(data => {
        if (data.success) {
          setCameraInfo(data);
        }
      })
      .catch(err => console.error('Failed to fetch camera info:', err));
  }, [enabled]);

  if (!enabled) {
    return (
      <div style={{ 
        padding: 12, 
        margin: 10, 
        background: '#1f2937', 
        color: 'white', 
        borderRadius: 8 
      }}>
        <p style={{ margin: 0, fontSize: 12, color: '#9ca3af' }}>
          Camera feeds disabled
        </p>
      </div>
    );
  }

  return (
    <div style={{ 
      padding: 12, 
      margin: 10, 
      background: '#1f2937', 
      color: 'white', 
      borderRadius: 8 
    }}>
      <h3 style={{ margin: '0 0 12px 0', fontSize: 16 }}>
        🎥 Robot Camera Feeds
      </h3>
      
      {cameraInfo && !cameraInfo.initialized && (
        <p style={{ color: '#ef4444', fontSize: 12 }}>
          ⚠️ Cameras not initialized. Check bridge logs.
        </p>
      )}
      
      <div style={{ 
        display: 'flex', 
        flexWrap: 'wrap', 
        gap: 8,
        justifyContent: 'center'
      }}>
        <CameraFeed
          cameraName="gripper_cam"
          title="Gripper Camera"
          width={320}
          height={240}
          refreshRate={100}
        />
        <CameraFeed
          cameraName="top_cam"
          title="Top Camera"
          width={320}
          height={240}
          refreshRate={100}
        />
      </div>
      
      <div style={{ 
        marginTop: 12, 
        fontSize: 11, 
        color: '#6b7280',
        textAlign: 'center'
      }}>
        Feeds update at 10 FPS • Resolution: 320x240
      </div>
    </div>
  );
}