import { useEffect, useState, useRef, useCallback } from 'react';
import {
  useBridgeWebSocket,
  CameraFrames,
  ReasoningMessage,
  NextActionMessage,
  ExecutionResultMessage,
  TaskCompleteMessage,
  ErrorMessage,
  RobotStatus,
} from '../../hooks/useBridgeWebSocket';

// Props for ALOHAControl
interface ALOHAControlProps {
  onRobotStatusChange?: (status: RobotStatus) => void;
}

// Chat message type for the UI
interface ChatMessage {
  id: string;
  timestamp: string;
  type: 'user' | 'assistant' | 'system' | 'action' | 'result' | 'connection' | 'error';
  content: string;
  metadata?: {
    step?: number;
    isError?: boolean;
    images?: string[];
  };
}

// Chat bubble colors
const CHAT_COLORS = {
  user: { background: '#3b82f6', text: '#ffffff' },
  assistant: { background: '#374151', text: '#e5e7eb' },
  system: { background: 'transparent', text: '#9ca3af' },
  action: { background: '#1e3a5f', text: '#60a5fa' },
  connection: { background: '#1e3a5f', text: '#60a5fa' },
  error: { background: '#7f1d1d', text: '#f87171' },
  result: {
    success: { background: '#064e3b', text: '#34d399' },
    error: { background: '#7f1d1d', text: '#f87171' },
  },
};

export function ALOHAControl({ onRobotStatusChange }: ALOHAControlProps) {
  const [taskStatus, setTaskStatus] = useState('Ready');
  const chatEndRef = useRef<HTMLDivElement>(null);

  // Chat messages state
  const [chatMessages, setChatMessages] = useState<ChatMessage[]>([]);

  // Task state
  const [taskInput, setTaskInput] = useState('');
  const [isExecuting, setIsExecuting] = useState(false);
  const [currentStep, setCurrentStep] = useState(0);
  const [cameraImages, setCameraImages] = useState<CameraFrames | null>(null);

  // Add a message to chat
  const addChatMessage = useCallback((message: Omit<ChatMessage, 'id' | 'timestamp'>) => {
    const newMessage: ChatMessage = {
      ...message,
      id: `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date().toLocaleTimeString(),
    };
    setChatMessages(prev => [...prev, newMessage]);
  }, []);

  // WebSocket message handlers
  const handleReasoning = useCallback((msg: ReasoningMessage) => {
    setCurrentStep(msg.step);
    setTaskStatus(`Step ${msg.step}`);
    addChatMessage({
      type: 'assistant',
      content: msg.text,
      metadata: { step: msg.step },
    });
  }, [addChatMessage]);

  const handleNextAction = useCallback((msg: NextActionMessage) => {
    addChatMessage({
      type: 'action',
      content: `${msg.function}(${JSON.stringify(msg.args)})`,
      metadata: { step: msg.step },
    });
  }, [addChatMessage]);

  const handleExecutionResult = useCallback((msg: ExecutionResultMessage) => {
    addChatMessage({
      type: 'result',
      content: msg.message || (msg.success ? 'Success' : 'Failed'),
      metadata: { step: msg.step, isError: !msg.success },
    });
  }, [addChatMessage]);

  const handleTaskComplete = useCallback((msg: TaskCompleteMessage) => {
    setIsExecuting(false);
    setTaskStatus('Complete');
    addChatMessage({
      type: 'system',
      content: `Task completed in ${msg.total_steps} step${msg.total_steps !== 1 ? 's' : ''}`,
    });
  }, [addChatMessage]);

  const handleError = useCallback((msg: ErrorMessage) => {
    setIsExecuting(false);
    setTaskStatus('Error');
    addChatMessage({
      type: 'error',
      content: `${msg.code}: ${msg.message}`,
      metadata: { isError: true },
    });
  }, [addChatMessage]);

  const handleCameraFrame = useCallback((frames: CameraFrames) => {
    setCameraImages(frames);
  }, []);

  const handleRobotStatus = useCallback((status: RobotStatus) => {
    onRobotStatusChange?.(status);
    // Don't add chat message for every status update, just log
    console.log('[ALOHAControl] Robot status:', status);
  }, [onRobotStatusChange]);

  // Initialize WebSocket
  const {
    isConnected,
    connectionState,
    robotStatus,
    sendTaskRequest,
    sendRobotConnect,
    sendRobotDisconnect,
  } = useBridgeWebSocket({
    onReasoning: handleReasoning,
    onNextAction: handleNextAction,
    onExecutionResult: handleExecutionResult,
    onTaskComplete: handleTaskComplete,
    onError: handleError,
    onCameraFrame: handleCameraFrame,
    onRobotStatus: handleRobotStatus,
  });

  // Derive robot connected state
  const robotConnected = robotStatus?.connected ?? false;

  // Show connection status in chat when it changes
  useEffect(() => {
    if (connectionState === 'connected') {
      addChatMessage({
        type: 'connection',
        content: `WebSocket connected\nRobot: ${robotConnected ? 'Ready' : 'Not connected'}`,
      });
    }
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [connectionState]);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (chatEndRef.current) {
      chatEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [chatMessages]);

  // Handle task submission
  const handleTaskSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!taskInput.trim() || isExecuting || !robotConnected) return;

    const task = taskInput.trim();
    setTaskInput('');
    setIsExecuting(true);
    setCurrentStep(0);
    setTaskStatus('Sending task...');

    // Add user message
    addChatMessage({
      type: 'user',
      content: task,
    });

    // Send via WebSocket
    sendTaskRequest(task);
  };

  // Handle robot connect/disconnect
  const handleRobotToggle = () => {
    if (robotConnected) {
      sendRobotDisconnect();
    } else {
      sendRobotConnect();
    }
  };

  // Clear chat
  const clearChat = () => {
    setChatMessages([]);
    setCameraImages(null);
    setCurrentStep(0);
    setTaskStatus('Ready');
  };

  // Chat bubble component
  const ChatBubble = ({ message }: { message: ChatMessage }) => {
    const isUser = message.type === 'user';
    const isSystem = message.type === 'system';
    const isAction = message.type === 'action';
    const isResult = message.type === 'result';
    const isConnection = message.type === 'connection';
    const isError = message.type === 'error' || message.metadata?.isError;

    let background: string;
    let textColor = '#e5e7eb';

    if (isResult) {
      const colors = isError ? CHAT_COLORS.result.error : CHAT_COLORS.result.success;
      background = colors.background;
      textColor = colors.text;
    } else if (isError) {
      background = CHAT_COLORS.error.background;
      textColor = CHAT_COLORS.error.text;
    } else {
      const style = CHAT_COLORS[message.type as keyof typeof CHAT_COLORS];
      if (typeof style === 'object' && 'background' in style) {
        background = style.background;
        textColor = style.text;
      } else {
        background = '#374151';
      }
    }

    return (
      <div style={{
        display: 'flex',
        justifyContent: isUser ? 'flex-end' : isSystem ? 'center' : 'flex-start',
        marginBottom: 8,
      }}>
        <div style={{
          maxWidth: isSystem ? '100%' : isConnection ? '100%' : '85%',
          padding: isAction ? '8px 12px' : '10px 14px',
          borderRadius: 12,
          background,
          color: textColor,
          fontSize: isAction ? 12 : 14,
          lineHeight: 1.5,
          fontFamily: isAction ? 'monospace' : 'inherit',
          whiteSpace: isConnection ? 'pre-line' : 'normal',
          width: isConnection ? '100%' : 'auto',
        }}>
          {isAction && (
            <span style={{ opacity: 0.6, marginRight: 8 }}>Action:</span>
          )}
          {isResult && !isError && (
            <span style={{ marginRight: 8 }}>Success:</span>
          )}
          {isResult && isError && (
            <span style={{ marginRight: 8 }}>Failed:</span>
          )}
          {isConnection && (
            <span style={{ opacity: 0.8, marginRight: 8, fontWeight: 'bold' }}>System:</span>
          )}
          <span style={{ fontStyle: isSystem ? 'italic' : 'normal' }}>
            {message.content}
          </span>
          {message.metadata?.step && (
            <div style={{ fontSize: 10, opacity: 0.5, marginTop: 4 }}>
              Step {message.metadata.step} • {message.timestamp}
            </div>
          )}
          {!message.metadata?.step && (
            <div style={{ fontSize: 10, opacity: 0.5, marginTop: 4 }}>
              {message.timestamp}
            </div>
          )}
        </div>
      </div>
    );
  };

  return (
    <div style={{ padding: 12, margin: 10, background: '#1f2937', color: 'white', borderRadius: 8 }}>
      {/* Header */}
      <div style={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: 12,
        paddingBottom: 8,
        borderBottom: '1px solid #374151'
      }}>
        <h3 style={{ margin: 0 }}>Robot Control</h3>
        <div style={{ fontSize: 12, display: 'flex', gap: 12, alignItems: 'center' }}>
          <span style={{ color: isConnected ? '#10b981' : '#ef4444' }}>
            WS: {connectionState}
          </span>
          <span style={{ color: robotConnected ? '#10b981' : '#ef4444' }}>
            Robot: {robotConnected ? 'Ready' : 'Disconnected'}
          </span>
          <span style={{ opacity: 0.6 }}>{taskStatus}</span>
        </div>
      </div>

      {/* Robot Connect/Disconnect Button */}
      <div style={{ marginBottom: 12 }}>
        <button
          onClick={handleRobotToggle}
          disabled={!isConnected}
          style={{
            padding: '8px 16px',
            borderRadius: 6,
            border: 'none',
            background: robotConnected ? '#ef4444' : '#10b981',
            color: 'white',
            cursor: !isConnected || isExecuting ? 'not-allowed' : 'pointer',
            opacity: !isConnected || isExecuting ? 0.5 : 1,
            fontSize: 12,
            fontWeight: 'bold',
          }}
        >
          {robotConnected ? 'Disconnect Robot' : 'Connect Robot'}
        </button>
      </div>

      {/* Chat Display */}
      <div style={{
        background: '#111827',
        borderRadius: 6,
        marginBottom: 12,
        height: 350,
        overflowY: 'auto',
        padding: 12,
      }}>
        {chatMessages.length === 0 ? (
          <div style={{
            textAlign: 'center',
            opacity: 0.5,
            padding: 40,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            height: '100%'
          }}>
            <div style={{ fontSize: 24, marginBottom: 8 }}>
              {!isConnected
                ? 'Connecting to bridge...'
                : robotConnected
                  ? 'Enter a task below to start'
                  : 'Connect to robot first'}
            </div>
            <div style={{ fontSize: 12 }}>
              {isConnected && robotConnected
                ? 'Example: "Pick up the red cube and place it in the bowl"'
                : isConnected
                  ? 'Use the Connect Robot button above'
                  : 'WebSocket connecting...'}
            </div>
          </div>
        ) : (
          chatMessages.map((msg) => (
            <ChatBubble key={msg.id} message={msg} />
          ))
        )}
        <div ref={chatEndRef} />
      </div>

      {/* Camera Preview (if images available) - Dynamic for any number of cameras */}
      {cameraImages && Object.keys(cameraImages).length > 0 && (
        <div style={{
          display: 'flex',
          gap: 8,
          marginBottom: 12,
          background: '#111827',
          padding: 8,
          borderRadius: 6,
          flexWrap: 'wrap'
        }}>
          {Object.entries(cameraImages).map(([cameraName, imageData]) => (
            imageData && (
              <div key={cameraName} style={{ flex: 1, minWidth: 150 }}>
                <div style={{ fontSize: 10, opacity: 0.6, marginBottom: 4 }}>
                  {cameraName.replace('_', ' ').replace(/\b\w/g, c => c.toUpperCase())}
                </div>
                <img
                  src={`data:image/jpeg;base64,${imageData}`}
                  alt={cameraName}
                  style={{
                    width: '100%',
                    borderRadius: 4,
                    border: '1px solid #374151',
                  }}
                />
              </div>
            )
          ))}
        </div>
      )}

      {/* Task Input Bar */}
      <form onSubmit={handleTaskSubmit} style={{ display: 'flex', gap: 8 }}>
        <input
          type="text"
          value={taskInput}
          onChange={(e) => setTaskInput(e.target.value)}
          placeholder={robotConnected ? "Enter task (e.g., Pick up the red cube)" : "Connect to robot first..."}
          disabled={isExecuting || !robotConnected}
          style={{
            flex: 1,
            padding: '12px 16px',
            borderRadius: 24,
            border: '1px solid #374151',
            background: '#111827',
            color: 'white',
            fontSize: 14,
            outline: 'none',
            opacity: robotConnected ? 1 : 0.5,
          }}
        />
        <button
          type="submit"
          disabled={isExecuting || !taskInput.trim() || !robotConnected}
          style={{
            padding: '12px 24px',
            borderRadius: 24,
            border: 'none',
            background: isExecuting || !robotConnected ? '#4b5563' : '#3b82f6',
            color: 'white',
            cursor: isExecuting || !robotConnected ? 'not-allowed' : 'pointer',
            fontWeight: 'bold',
            fontSize: 14,
          }}
        >
          {isExecuting ? '...' : 'Send'}
        </button>
        {chatMessages.length > 0 && (
          <button
            type="button"
            onClick={clearChat}
            disabled={isExecuting}
            style={{
              padding: '12px 16px',
              borderRadius: 24,
              border: '1px solid #374151',
              background: 'transparent',
              color: '#9ca3af',
              cursor: isExecuting ? 'not-allowed' : 'pointer',
              fontSize: 14,
            }}
          >
            Clear
          </button>
        )}
      </form>

      {/* Connection info */}
      <div style={{ fontSize: 10, opacity: 0.4, marginTop: 8, textAlign: 'center' }}>
        WebSocket: {connectionState} | Step: {currentStep}
      </div>
    </div>
  );
}
