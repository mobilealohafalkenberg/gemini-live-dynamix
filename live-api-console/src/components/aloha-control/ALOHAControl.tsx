import { useEffect, useState, useRef } from 'react';

const ER_BRIDGE_ENDPOINT = process.env.REACT_APP_ER_BRIDGE_ENDPOINT || 'http://localhost:8082';

// ER Bridge Response Types
interface ERAction {
  function: string;
  args: Record<string, any>;
}

interface ERExecutionResult {
  success: boolean;
  function: string;
  args: Record<string, any>;
  message?: string;
  error?: string;
  new_position?: number[];
  gripper_state?: string;
}

interface ERResponse {
  success: boolean;
  conversation_id: string;
  step: number;
  reasoning: string;
  next_action: ERAction | null;
  execution_result: ERExecutionResult | null;
  images: string[];
  verification_check: string;
  task_complete: boolean;
  error?: string;
}

interface ERStep {
  step: number;
  reasoning: string;
  action: ERAction | null;
  result: ERExecutionResult | null;
  verification: string;
  images: string[];
  timestamp: string;
}

// Chat message type for the UI
interface ChatMessage {
  id: string;
  timestamp: string;
  type: 'user' | 'assistant' | 'system' | 'action' | 'result' | 'connection';
  content: string;
  metadata?: {
    step?: number;
    action?: ERAction;
    result?: ERExecutionResult;
    isError?: boolean;
    images?: string[];
    connectionInfo?: ConnectionInfo;
  };
}

// Connection info from ER Bridge
interface ConnectionInfo {
  connected_arms: string[];
  arm_count: number;
  cameras: string[];
  gemini_api_ready: boolean;
}

// Props for ALOHAControl
interface ALOHAControlProps {
  robotConnected: boolean;
  connectionInfo?: ConnectionInfo;
}

// Chat bubble colors
const CHAT_COLORS = {
  user: { background: '#3b82f6', text: '#ffffff' },
  assistant: { background: '#374151', text: '#e5e7eb' },
  system: { background: 'transparent', text: '#9ca3af' },
  action: { background: '#1e3a5f', text: '#60a5fa' },
  connection: { background: '#1e3a5f', text: '#60a5fa' },
  result: {
    success: { background: '#064e3b', text: '#34d399' },
    error: { background: '#7f1d1d', text: '#f87171' },
  },
};

export function ALOHAControl({ robotConnected, connectionInfo }: ALOHAControlProps) {
  const [taskStatus, setTaskStatus] = useState('Ready');
  const chatEndRef = useRef<HTMLDivElement>(null);

  // Chat messages state
  const [chatMessages, setChatMessages] = useState<ChatMessage[]>([]);

  // ER Bridge State
  const [erTaskInput, setErTaskInput] = useState('');
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  const [erConversationId, setErConversationId] = useState<string | null>(null);
  const [erSteps, setErSteps] = useState<ERStep[]>([]);
  const [erIsExecuting, setErIsExecuting] = useState(false);
  const [erTaskComplete, setErTaskComplete] = useState(false);
  const [erCurrentTask, setErCurrentTask] = useState<string>('');
  const [erCameraImages, setErCameraImages] = useState<string[]>([]);

  // Track if we've shown the connection message
  const [connectionMessageShown, setConnectionMessageShown] = useState(false);

  // Show connection info in chat when robot connects
  useEffect(() => {
    if (robotConnected && connectionInfo && !connectionMessageShown) {
      const connectionMessage: ChatMessage = {
        id: 'connection-info',
        timestamp: new Date().toLocaleTimeString(),
        type: 'connection',
        content: formatConnectionInfo(connectionInfo),
        metadata: { connectionInfo }
      };
      setChatMessages(prev => [connectionMessage, ...prev.filter(m => m.id !== 'connection-info')]);
      setConnectionMessageShown(true);
    } else if (!robotConnected && connectionMessageShown) {
      // Reset when disconnected
      setConnectionMessageShown(false);
    }
  }, [robotConnected, connectionInfo, connectionMessageShown]);

  // Format connection info for display
  const formatConnectionInfo = (info: ConnectionInfo): string => {
    const lines = [
      'Connected to ER Bridge',
      `Arms: ${info.connected_arms.length > 0 ? info.connected_arms.join(', ') : 'None detected'}`,
      `Cameras: ${info.cameras.length > 0 ? info.cameras.join(', ') : 'None'}`,
      `Gemini ER API: ${info.gemini_api_ready ? 'Ready' : 'Not configured'}`
    ];
    return lines.join('\n');
  };

  // Convert ER steps to chat messages
  useEffect(() => {
    const messages: ChatMessage[] = [];

    // Keep connection message at the top if it exists
    const existingConnectionMsg = chatMessages.find(m => m.id === 'connection-info');
    if (existingConnectionMsg) {
      messages.push(existingConnectionMsg);
    }

    // Add user task as message
    if (erCurrentTask) {
      messages.push({
        id: 'task-input',
        timestamp: new Date().toLocaleTimeString(),
        type: 'user',
        content: erCurrentTask,
      });
    }

    // Convert each step to chat messages
    erSteps.forEach((step) => {
      // Add reasoning
      if (step.reasoning) {
        messages.push({
          id: `step-${step.step}-reasoning`,
          timestamp: step.timestamp,
          type: 'assistant',
          content: step.reasoning,
          metadata: { step: step.step, images: step.images }
        });
      }

      // Add action
      if (step.action) {
        messages.push({
          id: `step-${step.step}-action`,
          timestamp: step.timestamp,
          type: 'action',
          content: `${step.action.function}(${JSON.stringify(step.action.args)})`,
          metadata: { action: step.action }
        });
      }

      // Add result
      if (step.result) {
        messages.push({
          id: `step-${step.step}-result`,
          timestamp: step.timestamp,
          type: 'result',
          content: step.result.message || step.result.error || 'Completed',
          metadata: { result: step.result, isError: !step.result.success }
        });
      }
    });

    // Add completion message
    if (erTaskComplete) {
      messages.push({
        id: 'task-complete',
        timestamp: new Date().toLocaleTimeString(),
        type: 'system',
        content: 'Task completed',
      });
    }

    setChatMessages(messages);
  }, [erSteps, erCurrentTask, erTaskComplete, chatMessages]);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (chatEndRef.current) {
      chatEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [chatMessages]);

  // ER Bridge API Functions
  const sendERTask = async (prompt: string) => {
    if (!robotConnected) {
      // Add error message to chat
      const errorMsg: ChatMessage = {
        id: `error-${Date.now()}`,
        timestamp: new Date().toLocaleTimeString(),
        type: 'system',
        content: 'Please connect to the robot first',
        metadata: { isError: true }
      };
      setChatMessages(prev => [...prev, errorMsg]);
      return;
    }

    setErIsExecuting(true);
    setErTaskComplete(false);
    setErSteps([]);
    setErCurrentTask(prompt);
    setErCameraImages([]);
    setTaskStatus('Sending task...');

    try {
      const res = await fetch(`${ER_BRIDGE_ENDPOINT}/robotics-er-request`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ prompt }),
      });

      const data: ERResponse = await res.json();

      if (!data.success) {
        console.error('ER Bridge error:', data.error);
        const errorMsg: ChatMessage = {
          id: `error-${Date.now()}`,
          timestamp: new Date().toLocaleTimeString(),
          type: 'result',
          content: data.error || 'Unknown error',
          metadata: { isError: true }
        };
        setChatMessages(prev => [...prev, errorMsg]);
        setErIsExecuting(false);
        setTaskStatus('Error');
        return;
      }

      setErConversationId(data.conversation_id);
      setTaskStatus(`Step ${data.step}`);

      // Add step to history
      const newStep: ERStep = {
        step: data.step,
        reasoning: data.reasoning,
        action: data.next_action,
        result: data.execution_result,
        verification: data.verification_check,
        images: data.images,
        timestamp: new Date().toLocaleTimeString(),
      };
      setErSteps([newStep]);

      // Update camera images
      if (data.images && data.images.length > 0) {
        setErCameraImages(data.images);
      }

      // Check if task is complete
      if (data.task_complete) {
        setErTaskComplete(true);
        setErIsExecuting(false);
        setErConversationId(null);
        setTaskStatus('Complete');
      } else if (data.execution_result) {
        // Auto-continue with feedback
        await sendERFeedback(data.conversation_id, data.execution_result);
      } else {
        setErIsExecuting(false);
        setTaskStatus('Ready');
      }
    } catch (e: any) {
      console.error('ER Bridge fetch error:', e?.message || e);
      const errorMsg: ChatMessage = {
        id: `error-${Date.now()}`,
        timestamp: new Date().toLocaleTimeString(),
        type: 'result',
        content: `Connection error: ${e?.message || 'Failed to reach ER Bridge'}`,
        metadata: { isError: true }
      };
      setChatMessages(prev => [...prev, errorMsg]);
      setErIsExecuting(false);
      setTaskStatus('Error');
    }
  };

  const sendERFeedback = async (conversationId: string, executionResult: ERExecutionResult) => {
    try {
      setTaskStatus('Processing feedback...');
      const res = await fetch(`${ER_BRIDGE_ENDPOINT}/robotics-er-request`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          conversation_id: conversationId,
          execution_result: executionResult,
        }),
      });

      const data: ERResponse = await res.json();

      if (!data.success) {
        console.error('ER Bridge feedback error:', data.error);
        setErIsExecuting(false);
        setTaskStatus('Error');
        return;
      }

      setTaskStatus(`Step ${data.step}`);

      // Add step to history
      const newStep: ERStep = {
        step: data.step,
        reasoning: data.reasoning,
        action: data.next_action,
        result: data.execution_result,
        verification: data.verification_check,
        images: data.images,
        timestamp: new Date().toLocaleTimeString(),
      };
      setErSteps(prev => [...prev, newStep]);

      // Update camera images
      if (data.images && data.images.length > 0) {
        setErCameraImages(data.images);
      }

      // Check if task is complete
      if (data.task_complete) {
        setErTaskComplete(true);
        setErIsExecuting(false);
        setErConversationId(null);
        setTaskStatus('Complete');
      } else if (data.execution_result) {
        // Auto-continue with feedback (recursive)
        await sendERFeedback(data.conversation_id, data.execution_result);
      } else {
        setErIsExecuting(false);
        setTaskStatus('Ready');
      }
    } catch (e: any) {
      console.error('ER Bridge feedback fetch error:', e?.message || e);
      setErIsExecuting(false);
      setTaskStatus('Error');
    }
  };

  const handleERTaskSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!erTaskInput.trim() || erIsExecuting) return;

    const task = erTaskInput.trim();
    setErTaskInput('');
    await sendERTask(task);
  };

  const clearChat = () => {
    setErSteps([]);
    setErConversationId(null);
    setErTaskComplete(false);
    setErCurrentTask('');
    setErCameraImages([]);
    // Keep connection message, clear everything else
    setChatMessages(prev => prev.filter(m => m.id === 'connection-info'));
  };

  // Chat bubble component
  const ChatBubble = ({ message }: { message: ChatMessage }) => {
    const isUser = message.type === 'user';
    const isSystem = message.type === 'system';
    const isAction = message.type === 'action';
    const isResult = message.type === 'result';
    const isConnection = message.type === 'connection';
    const isError = message.metadata?.isError;

    let background = CHAT_COLORS[message.type as keyof typeof CHAT_COLORS];
    let textColor = '#e5e7eb';

    if (isResult) {
      background = isError ? CHAT_COLORS.result.error : CHAT_COLORS.result.success;
      textColor = (background as any).text;
    } else if (typeof background === 'object' && 'background' in background) {
      textColor = background.text;
      background = background.background as any;
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
          background: typeof background === 'string' ? background : '#374151',
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
          {isResult && (
            <span style={{ marginRight: 8 }}>{isError ? 'Failed:' : 'Success:'}</span>
          )}
          {isConnection && (
            <span style={{ opacity: 0.8, marginRight: 8, fontWeight: 'bold' }}>System:</span>
          )}
          <span style={{ fontStyle: isSystem ? 'italic' : 'normal' }}>
            {message.content}
          </span>
          <div style={{ fontSize: 10, opacity: 0.5, marginTop: 4 }}>
            {message.timestamp}
          </div>
        </div>
      </div>
    );
  };

  return (
    <div style={{ padding: 12, margin: 10, background: '#1f2937', color: 'white', borderRadius: 8 }}>
      {/* Minimal Header */}
      <div style={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        marginBottom: 12,
        paddingBottom: 8,
        borderBottom: '1px solid #374151'
      }}>
        <h3 style={{ margin: 0 }}>Robot Control</h3>
        <div style={{ fontSize: 12, display: 'flex', gap: 12 }}>
          <span style={{ color: robotConnected ? '#10b981' : '#ef4444' }}>
            {robotConnected ? 'Connected' : 'Disconnected'}
          </span>
          <span style={{ opacity: 0.6 }}>{taskStatus}</span>
        </div>
      </div>

      {/* Chat Display */}
      <div style={{
        background: '#111827',
        borderRadius: 6,
        marginBottom: 12,
        height: 400,
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
              {robotConnected ? 'Enter a task below to start' : 'Connect to robot first'}
            </div>
            <div style={{ fontSize: 12 }}>
              {robotConnected
                ? 'Example: "Pick up the red cube and place it in the bowl"'
                : 'Use the Connect Robot button below'}
            </div>
          </div>
        ) : (
          chatMessages.map((msg) => (
            <ChatBubble key={msg.id} message={msg} />
          ))
        )}
        <div ref={chatEndRef} />
      </div>

      {/* Camera Preview (if images available) */}
      {erCameraImages.length > 0 && (
        <div style={{
          display: 'flex',
          gap: 8,
          marginBottom: 12,
          background: '#111827',
          padding: 8,
          borderRadius: 6
        }}>
          {erCameraImages.map((img, idx) => (
            img && (
              <div key={idx} style={{ flex: 1 }}>
                <div style={{ fontSize: 10, opacity: 0.6, marginBottom: 4 }}>
                  {idx === 0 ? 'Gripper Cam' : 'Top Cam'}
                </div>
                <img
                  src={`data:image/jpeg;base64,${img}`}
                  alt={idx === 0 ? 'Gripper camera' : 'Top camera'}
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
      <form onSubmit={handleERTaskSubmit} style={{ display: 'flex', gap: 8 }}>
        <input
          type="text"
          value={erTaskInput}
          onChange={(e) => setErTaskInput(e.target.value)}
          placeholder={robotConnected ? "Enter task (e.g., Pick up the red cube)" : "Connect to robot first..."}
          disabled={erIsExecuting || !robotConnected}
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
          disabled={erIsExecuting || !erTaskInput.trim() || !robotConnected}
          style={{
            padding: '12px 24px',
            borderRadius: 24,
            border: 'none',
            background: erIsExecuting || !robotConnected ? '#4b5563' : '#3b82f6',
            color: 'white',
            cursor: erIsExecuting || !robotConnected ? 'not-allowed' : 'pointer',
            fontWeight: 'bold',
            fontSize: 14,
          }}
        >
          {erIsExecuting ? '...' : 'Send'}
        </button>
        {chatMessages.filter(m => m.id !== 'connection-info').length > 0 && (
          <button
            type="button"
            onClick={clearChat}
            style={{
              padding: '12px 16px',
              borderRadius: 24,
              border: '1px solid #374151',
              background: 'transparent',
              color: '#9ca3af',
              cursor: 'pointer',
              fontSize: 14,
            }}
          >
            Clear
          </button>
        )}
      </form>

      {/* Bridge endpoint info */}
      <div style={{ fontSize: 10, opacity: 0.4, marginTop: 8, textAlign: 'center' }}>
        ER Bridge: {ER_BRIDGE_ENDPOINT}
      </div>
    </div>
  );
}
