import { CameraFrames } from '../../hooks/useBridgeWebSocket';

interface CameraFeedProps {
  title: string;
  imageData?: string;  // base64 JPEG from WebSocket
  width?: number;
  height?: number;
  isConnected?: boolean;
}

export function CameraFeed({
  title,
  imageData,
  width = 320,
  height = 240,
  isConnected = false,
}: CameraFeedProps) {
  const imageUrl = imageData ? `data:image/jpeg;base64,${imageData}` : '';

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
        {title}
      </h4>

      {!isConnected && (
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
          Waiting for connection...
        </div>
      )}

      {isConnected && !imageData && (
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
          Waiting for camera frame...
        </div>
      )}

      {isConnected && imageData && (
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
      )}

      <div style={{
        marginTop: 4,
        fontSize: 10,
        color: isConnected && imageData ? '#10b981' : '#6b7280',
        textAlign: 'center'
      }}>
        {isConnected && imageData ? 'Live (WebSocket)' : 'No data'}
      </div>
    </div>
  );
}

interface DualCameraViewProps {
  frames: CameraFrames | null;
  isConnected?: boolean;
}

export function DualCameraView({ frames, isConnected = false }: DualCameraViewProps) {
  return (
    <div style={{
      padding: 12,
      margin: 10,
      background: '#1f2937',
      color: 'white',
      borderRadius: 8
    }}>
      <h3 style={{ margin: '0 0 12px 0', fontSize: 16, display: 'flex', alignItems: 'center', gap: 8 }}>
        Robot Camera Feeds
        {isConnected && (
          <span style={{
            fontSize: 10,
            color: '#10b981',
            fontWeight: 'normal',
            background: '#064e3b',
            padding: '2px 8px',
            borderRadius: 4
          }}>
            SYNCED
          </span>
        )}
      </h3>

      <div style={{
        display: 'flex',
        flexWrap: 'wrap',
        gap: 8,
        justifyContent: 'center'
      }}>
        <CameraFeed
          title="Gripper Camera"
          imageData={frames?.gripper_cam}
          width={320}
          height={240}
          isConnected={isConnected}
        />
        <CameraFeed
          title="Top Camera"
          imageData={frames?.top_cam}
          width={320}
          height={240}
          isConnected={isConnected}
        />
      </div>

      <div style={{
        marginTop: 12,
        fontSize: 11,
        color: '#6b7280',
        textAlign: 'center'
      }}>
        {isConnected
          ? 'Images synchronized with Gemini AI (updates on capture)'
          : 'Connecting to WebSocket...'}
      </div>
    </div>
  );
}
