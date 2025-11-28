import { useEffect, useRef, useState, useCallback } from 'react';

// Derive WebSocket URL from HTTP endpoint
const HTTP_ENDPOINT = process.env.REACT_APP_BRIDGE_ENDPOINT || 'http://localhost:8082';
const WS_URL = HTTP_ENDPOINT.replace(/^http/, 'ws') + '/ws';

// Message types from bridge
export interface WebSocketMessage {
  type: string;
  timestamp: number;
  sequence: number;
  payload: any;
}

// Dynamic camera frames - keys are camera names
export interface CameraFrames {
  [cameraName: string]: string;
}

export interface RobotStatus {
  connected: boolean;
  connected_arms: string[];
  cameras?: string[];
  message?: string;
}

export interface ConnectionInfo {
  cameras: string[];
  connected_arms: string[];
  robot_connected: boolean;
}

export interface ReasoningMessage {
  text: string;
}

export interface NextActionMessage {
  function: string;
  args: Record<string, any>;
  step: number;
}

export interface ExecutionResultMessage {
  function: string;
  result: Record<string, any>;
  step: number;
  success: boolean;
}

export interface StepUpdateMessage {
  step: number;
  max_steps: number;
}

export interface TaskCompleteMessage {
  success: boolean;
  summary?: string;
  reason?: string;
}

export interface ErrorMessage {
  code: string;
  message: string;
}

// Callback types for message handlers
export interface MessageHandlers {
  onConnectionInfo?: (info: ConnectionInfo) => void;
  onCameraFrame?: (frames: CameraFrames) => void;
  onRobotStatus?: (status: RobotStatus) => void;
  onReasoning?: (msg: ReasoningMessage) => void;
  onNextAction?: (msg: NextActionMessage) => void;
  onExecutionResult?: (msg: ExecutionResultMessage) => void;
  onStepUpdate?: (msg: StepUpdateMessage) => void;
  onTaskComplete?: (msg: TaskCompleteMessage) => void;
  onError?: (msg: ErrorMessage) => void;
}

export interface UseAgentWebSocketReturn {
  isConnected: boolean;
  connectionState: 'connecting' | 'connected' | 'disconnected' | 'error';
  cameraFrames: CameraFrames | null;
  robotStatus: RobotStatus | null;
  currentStep: StepUpdateMessage | null;
  send: (type: string, payload?: any) => void;
  sendTaskRequest: (prompt: string) => void;
  sendRobotConnect: () => void;
  sendRobotDisconnect: () => void;
}

export function useAgentWebSocket(handlers: MessageHandlers = {}): UseAgentWebSocketReturn {
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout>();
  const handlersRef = useRef(handlers);

  // Keep handlers ref up to date
  handlersRef.current = handlers;

  const [isConnected, setIsConnected] = useState(false);
  const [connectionState, setConnectionState] = useState<'connecting' | 'connected' | 'disconnected' | 'error'>('disconnected');
  const [cameraFrames, setCameraFrames] = useState<CameraFrames | null>(null);
  const [robotStatus, setRobotStatus] = useState<RobotStatus | null>(null);
  const [currentStep, setCurrentStep] = useState<StepUpdateMessage | null>(null);

  // Send a message over WebSocket
  const send = useCallback((type: string, payload: any = {}) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ type, payload }));
    } else {
      console.warn('[WebSocket] Cannot send - not connected');
    }
  }, []);

  // Convenience methods
  const sendTaskRequest = useCallback((prompt: string) => {
    setCurrentStep(null); // Reset step on new task
    send('task_request', { prompt });
  }, [send]);

  const sendRobotConnect = useCallback(() => {
    send('robot_connect');
  }, [send]);

  const sendRobotDisconnect = useCallback(() => {
    send('robot_disconnect');
  }, [send]);

  // Handle incoming messages
  const handleMessage = useCallback((event: MessageEvent) => {
    try {
      const msg: WebSocketMessage = JSON.parse(event.data);
      const { type, payload } = msg;

      switch (type) {
        case 'connection_info':
          handlersRef.current.onConnectionInfo?.(payload as ConnectionInfo);
          // Also update robot status from connection info
          setRobotStatus({
            connected: payload.robot_connected,
            connected_arms: payload.connected_arms,
            cameras: payload.cameras
          });
          break;

        case 'camera_frame':
          setCameraFrames(payload as CameraFrames);
          handlersRef.current.onCameraFrame?.(payload as CameraFrames);
          break;

        case 'robot_status':
          setRobotStatus(payload as RobotStatus);
          handlersRef.current.onRobotStatus?.(payload as RobotStatus);
          break;

        case 'reasoning':
          handlersRef.current.onReasoning?.(payload as ReasoningMessage);
          break;

        case 'next_action':
          handlersRef.current.onNextAction?.(payload as NextActionMessage);
          break;

        case 'execution_result':
          handlersRef.current.onExecutionResult?.(payload as ExecutionResultMessage);
          break;

        case 'step_update':
          setCurrentStep(payload as StepUpdateMessage);
          handlersRef.current.onStepUpdate?.(payload as StepUpdateMessage);
          break;

        case 'task_complete':
          handlersRef.current.onTaskComplete?.(payload as TaskCompleteMessage);
          break;

        case 'error':
          handlersRef.current.onError?.(payload as ErrorMessage);
          break;

        default:
          console.log('[WebSocket] Unknown message type:', type);
      }
    } catch (e) {
      console.error('[WebSocket] Failed to parse message:', e);
    }
  }, []);

  // Connect to WebSocket
  useEffect(() => {
    const connect = () => {
      if (wsRef.current?.readyState === WebSocket.OPEN) {
        return;
      }

      setConnectionState('connecting');
      console.log('[WebSocket] Connecting to', WS_URL);

      const ws = new WebSocket(WS_URL);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('[WebSocket] Connected');
        setIsConnected(true);
        setConnectionState('connected');
      };

      ws.onmessage = handleMessage;

      ws.onclose = (event) => {
        console.log('[WebSocket] Disconnected:', event.code, event.reason);
        setIsConnected(false);
        setConnectionState('disconnected');
        wsRef.current = null;

        // Reconnect after 3 seconds
        reconnectTimeoutRef.current = setTimeout(connect, 3000);
      };

      ws.onerror = (error) => {
        console.error('[WebSocket] Error:', error);
        setConnectionState('error');
      };
    };

    connect();

    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      if (wsRef.current) {
        wsRef.current.close(1000, 'Component unmount');
        wsRef.current = null;
      }
    };
  }, [handleMessage]);

  return {
    isConnected,
    connectionState,
    cameraFrames,
    robotStatus,
    currentStep,
    send,
    sendTaskRequest,
    sendRobotConnect,
    sendRobotDisconnect,
  };
}
