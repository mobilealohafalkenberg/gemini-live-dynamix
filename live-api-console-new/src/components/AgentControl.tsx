import { useEffect, useState, useRef, useCallback } from 'react';
import {
  useAgentWebSocket,
  CameraFrames,
  ReasoningMessage,
  NextActionMessage,
  ExecutionResultMessage,
  TaskCompleteMessage,
  ErrorMessage,
  StepUpdateMessage,
} from '../hooks/useAgentWebSocket';

// Chat message type for the UI
interface ChatMessage {
  id: string;
  timestamp: string;
  type: 'user' | 'reasoning' | 'action' | 'result' | 'system' | 'error';
  content: string;
  metadata?: {
    step?: number;
    success?: boolean;
  };
}

// Message colors
const COLORS = {
  user: { bg: '#3b82f6', text: '#ffffff' },
  reasoning: { bg: '#374151', text: '#d1d5db' },
  action: { bg: '#1e3a5f', text: '#60a5fa' },
  result: { bg: '#064e3b', text: '#34d399' },
  result_error: { bg: '#7f1d1d', text: '#f87171' },
  system: { bg: 'transparent', text: '#9ca3af' },
  error: { bg: '#7f1d1d', text: '#f87171' },
};

export function AgentControl() {
  const chatEndRef = useRef<HTMLDivElement>(null);

  // State
  const [chatMessages, setChatMessages] = useState<ChatMessage[]>([]);
  const [taskInput, setTaskInput] = useState('');
  const [isExecuting, setIsExecuting] = useState(false);
  const [cameraImages, setCameraImages] = useState<CameraFrames | null>(null);

  // Add message to chat
  const addMessage = useCallback((msg: Omit<ChatMessage, 'id' | 'timestamp'>) => {
    setChatMessages(prev => [...prev, {
      ...msg,
      id: `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
      timestamp: new Date().toLocaleTimeString(),
    }]);
  }, []);

  // Message handlers
  const handleReasoning = useCallback((msg: ReasoningMessage) => {
    addMessage({
      type: 'reasoning',
      content: msg.text,
    });
  }, [addMessage]);

  const handleNextAction = useCallback((msg: NextActionMessage) => {
    addMessage({
      type: 'action',
      content: `${msg.function}(${JSON.stringify(msg.args, null, 2)})`,
      metadata: { step: msg.step },
    });
  }, [addMessage]);

  const handleExecutionResult = useCallback((msg: ExecutionResultMessage) => {
    const resultStr = JSON.stringify(msg.result, null, 2);
    addMessage({
      type: 'result',
      content: `${msg.function}: ${msg.success ? 'Success' : 'Failed'}\n${resultStr}`,
      metadata: { step: msg.step, success: msg.success },
    });
  }, [addMessage]);

  const handleStepUpdate = useCallback((msg: StepUpdateMessage) => {
    // Just log, UI shows this in progress bar
    console.log(`[Step ${msg.step}/${msg.max_steps}]`);
  }, []);

  const handleTaskComplete = useCallback((msg: TaskCompleteMessage) => {
    setIsExecuting(false);
    addMessage({
      type: 'system',
      content: msg.success
        ? `Task completed: ${msg.summary || 'Success'}`
        : `Task failed: ${msg.reason || msg.summary || 'Unknown reason'}`,
      metadata: { success: msg.success },
    });
  }, [addMessage]);

  const handleError = useCallback((msg: ErrorMessage) => {
    setIsExecuting(false);
    addMessage({
      type: 'error',
      content: `${msg.code}: ${msg.message}`,
    });
  }, [addMessage]);

  const handleCameraFrame = useCallback((frames: CameraFrames) => {
    setCameraImages(frames);
  }, []);

  // WebSocket hook
  const {
    isConnected,
    connectionState,
    robotStatus,
    currentStep,
    sendTaskRequest,
    sendRobotConnect,
    sendRobotDisconnect,
  } = useAgentWebSocket({
    onReasoning: handleReasoning,
    onNextAction: handleNextAction,
    onExecutionResult: handleExecutionResult,
    onStepUpdate: handleStepUpdate,
    onTaskComplete: handleTaskComplete,
    onError: handleError,
    onCameraFrame: handleCameraFrame,
  });

  const robotConnected = robotStatus?.connected ?? false;

  // Auto-scroll chat
  useEffect(() => {
    chatEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [chatMessages]);

  // Submit task
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!taskInput.trim() || !robotConnected || isExecuting) return;

    addMessage({ type: 'user', content: taskInput });
    sendTaskRequest(taskInput);
    setIsExecuting(true);
    setTaskInput('');
  };

  // Toggle robot connection
  const handleToggleRobot = () => {
    if (robotConnected) {
      sendRobotDisconnect();
    } else {
      sendRobotConnect();
    }
  };

  // Clear chat
  const handleClear = () => {
    setChatMessages([]);
  };

  // Render message bubble
  const renderMessage = (msg: ChatMessage) => {
    let bgColor = COLORS[msg.type]?.bg || '#374151';
    let textColor = COLORS[msg.type]?.text || '#ffffff';

    // Special case for results
    if (msg.type === 'result' && msg.metadata?.success === false) {
      bgColor = COLORS.result_error.bg;
      textColor = COLORS.result_error.text;
    }

    return (
      <div
        key={msg.id}
        style={{
          marginBottom: 8,
          padding: '8px 12px',
          borderRadius: 8,
          backgroundColor: bgColor,
          color: textColor,
          fontFamily: 'Space Mono, monospace',
          fontSize: 13,
          whiteSpace: 'pre-wrap',
          wordBreak: 'break-word',
        }}
      >
        <div style={{ fontSize: 10, opacity: 0.7, marginBottom: 4 }}>
          {msg.timestamp} {msg.metadata?.step ? `[Step ${msg.metadata.step}]` : ''}
        </div>
        {msg.content}
      </div>
    );
  };

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      height: '100vh',
      backgroundColor: '#111111',
      color: '#ffffff',
      fontFamily: 'Space Mono, monospace',
    }}>
      {/* Header */}
      <div style={{
        padding: '12px 16px',
        borderBottom: '1px solid #333',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
          <span style={{ fontSize: 18, fontWeight: 'bold' }}>Gemini Robotics Agent</span>
          <span style={{
            padding: '2px 8px',
            borderRadius: 4,
            fontSize: 11,
            backgroundColor: connectionState === 'connected' ? '#064e3b' : '#7f1d1d',
            color: connectionState === 'connected' ? '#34d399' : '#f87171',
          }}>
            {connectionState}
          </span>
        </div>

        <div style={{ display: 'flex', gap: 8 }}>
          <button
            onClick={handleToggleRobot}
            disabled={!isConnected || isExecuting}
            style={{
              padding: '6px 12px',
              borderRadius: 4,
              border: 'none',
              cursor: isConnected && !isExecuting ? 'pointer' : 'not-allowed',
              opacity: isConnected && !isExecuting ? 1 : 0.5,
              backgroundColor: robotConnected ? '#7f1d1d' : '#064e3b',
              color: '#ffffff',
              fontFamily: 'inherit',
              fontSize: 12,
            }}
          >
            {robotConnected ? 'Disconnect' : 'Connect Robot'}
          </button>
          <button
            onClick={handleClear}
            style={{
              padding: '6px 12px',
              borderRadius: 4,
              border: '1px solid #444',
              cursor: 'pointer',
              backgroundColor: 'transparent',
              color: '#888',
              fontFamily: 'inherit',
              fontSize: 12,
            }}
          >
            Clear
          </button>
        </div>
      </div>

      {/* Main content */}
      <div style={{ flex: 1, display: 'flex', overflow: 'hidden' }}>
        {/* Chat panel */}
        <div style={{
          flex: 1,
          display: 'flex',
          flexDirection: 'column',
          borderRight: '1px solid #333',
        }}>
          {/* Progress bar */}
          {currentStep && isExecuting && (
            <div style={{ padding: '8px 16px', backgroundColor: '#1a1a1a' }}>
              <div style={{
                display: 'flex',
                justifyContent: 'space-between',
                fontSize: 11,
                marginBottom: 4,
                color: '#888',
              }}>
                <span>Step {currentStep.step} of {currentStep.max_steps}</span>
                <span>{Math.round((currentStep.step / currentStep.max_steps) * 100)}%</span>
              </div>
              <div style={{
                height: 4,
                backgroundColor: '#333',
                borderRadius: 2,
                overflow: 'hidden',
              }}>
                <div style={{
                  width: `${(currentStep.step / currentStep.max_steps) * 100}%`,
                  height: '100%',
                  backgroundColor: '#3b82f6',
                  transition: 'width 0.3s ease',
                }} />
              </div>
            </div>
          )}

          {/* Messages */}
          <div style={{
            flex: 1,
            overflowY: 'auto',
            padding: 16,
          }}>
            {chatMessages.length === 0 && (
              <div style={{
                textAlign: 'center',
                color: '#666',
                marginTop: 40,
              }}>
                <div style={{ fontSize: 14, marginBottom: 8 }}>No messages yet</div>
                <div style={{ fontSize: 12 }}>Connect the robot and enter a task to begin</div>
              </div>
            )}
            {chatMessages.map(renderMessage)}
            <div ref={chatEndRef} />
          </div>

          {/* Input */}
          <form onSubmit={handleSubmit} style={{
            padding: 16,
            borderTop: '1px solid #333',
            display: 'flex',
            gap: 8,
          }}>
            <input
              type="text"
              value={taskInput}
              onChange={(e) => setTaskInput(e.target.value)}
              placeholder={
                !isConnected ? 'Connecting...' :
                !robotConnected ? 'Connect robot first...' :
                isExecuting ? 'Task in progress...' :
                'Enter a task (e.g., "Pick up the red cube")'
              }
              disabled={!isConnected || !robotConnected || isExecuting}
              style={{
                flex: 1,
                padding: '10px 14px',
                borderRadius: 6,
                border: '1px solid #444',
                backgroundColor: '#1a1a1a',
                color: '#fff',
                fontFamily: 'inherit',
                fontSize: 13,
              }}
            />
            <button
              type="submit"
              disabled={!isConnected || !robotConnected || isExecuting || !taskInput.trim()}
              style={{
                padding: '10px 20px',
                borderRadius: 6,
                border: 'none',
                backgroundColor: '#3b82f6',
                color: '#fff',
                fontFamily: 'inherit',
                fontSize: 13,
                cursor: 'pointer',
                opacity: (!isConnected || !robotConnected || isExecuting || !taskInput.trim()) ? 0.5 : 1,
              }}
            >
              {isExecuting ? 'Running...' : 'Send'}
            </button>
          </form>
        </div>

        {/* Camera panel */}
        <div style={{
          width: 400,
          padding: 16,
          overflowY: 'auto',
          backgroundColor: '#0a0a0a',
        }}>
          <div style={{
            fontSize: 12,
            color: '#888',
            marginBottom: 12,
            textTransform: 'uppercase',
          }}>
            Camera Feeds
          </div>

          {(!cameraImages || Object.keys(cameraImages).length === 0) ? (
            <div style={{
              textAlign: 'center',
              color: '#555',
              padding: 40,
              border: '1px dashed #333',
              borderRadius: 8,
            }}>
              No camera frames
            </div>
          ) : (
            <div style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
              {Object.entries(cameraImages).map(([name, data]) => (
                <div key={name}>
                  <div style={{
                    fontSize: 10,
                    color: '#666',
                    marginBottom: 4,
                    textTransform: 'capitalize',
                  }}>
                    {name.replace(/_/g, ' ')}
                  </div>
                  <img
                    src={`data:image/jpeg;base64,${data}`}
                    alt={name}
                    style={{
                      width: '100%',
                      borderRadius: 6,
                      border: '1px solid #333',
                    }}
                  />
                </div>
              ))}
            </div>
          )}

          {/* Robot status */}
          <div style={{ marginTop: 24 }}>
            <div style={{
              fontSize: 12,
              color: '#888',
              marginBottom: 8,
              textTransform: 'uppercase',
            }}>
              Robot Status
            </div>
            <div style={{
              padding: 12,
              backgroundColor: '#1a1a1a',
              borderRadius: 6,
              fontSize: 11,
            }}>
              <div>Connected: {robotConnected ? 'Yes' : 'No'}</div>
              <div>Arms: {robotStatus?.connected_arms?.join(', ') || 'None'}</div>
              <div>Cameras: {robotStatus?.cameras?.join(', ') || 'None'}</div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
