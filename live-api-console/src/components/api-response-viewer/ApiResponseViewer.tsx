import { useState, useEffect, useRef } from 'react';

interface ApiMessage {
  timestamp: string;
  type: string;
  data: any;
}

interface ApiResponseViewerProps {
  messages: ApiMessage[];
  onClear: () => void;
}

export function ApiResponseViewer({ messages, onClear }: ApiResponseViewerProps) {
  const [isExpanded, setIsExpanded] = useState(false);
  const logEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (isExpanded && logEndRef.current) {
      logEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, isExpanded]);

  return (
    <div style={{
      marginTop: 12,
      background: '#111827',
      borderRadius: 8,
      border: '1px solid #374151'
    }}>
      {/* Header */}
      <div
        style={{
          padding: 12,
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          cursor: 'pointer',
          borderBottom: isExpanded ? '1px solid #374151' : 'none'
        }}
        onClick={() => setIsExpanded(!isExpanded)}
      >
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <span style={{ fontSize: 18 }}>{isExpanded ? '▼' : '▶'}</span>
          <h4 style={{ margin: 0 }}>📡 API Response Log</h4>
          <span style={{
            fontSize: 12,
            opacity: 0.6,
            background: '#374151',
            padding: '2px 8px',
            borderRadius: 12
          }}>
            {messages.length} messages
          </span>
        </div>
        <button
          onClick={(e) => {
            e.stopPropagation();
            onClear();
          }}
          style={{
            padding: '4px 12px',
            fontSize: 12,
            background: '#ef4444',
            border: 'none',
            borderRadius: 4,
            color: 'white',
            cursor: 'pointer'
          }}
        >
          Clear
        </button>
      </div>

      {/* Message Log */}
      {isExpanded && (
        <div style={{
          maxHeight: 400,
          overflowY: 'auto',
          padding: 12,
          fontFamily: 'monospace',
          fontSize: 12
        }}>
          {messages.length === 0 ? (
            <div style={{ opacity: 0.5, textAlign: 'center', padding: 20 }}>
              No messages yet. Start a conversation to see API responses.
            </div>
          ) : (
            messages.map((msg, idx) => (
              <div
                key={idx}
                style={{
                  marginBottom: 12,
                  padding: 12,
                  background: '#1f2937',
                  borderRadius: 6,
                  borderLeft: `3px solid ${getTypeColor(msg.type)}`
                }}
              >
                {/* Message Header */}
                <div style={{
                  display: 'flex',
                  justifyContent: 'space-between',
                  marginBottom: 8,
                  opacity: 0.7,
                  fontSize: 11
                }}>
                  <span style={{ color: getTypeColor(msg.type), fontWeight: 'bold' }}>
                    {msg.type.toUpperCase()}
                  </span>
                  <span>{msg.timestamp}</span>
                </div>

                {/* JSON Content */}
                <pre style={{
                  margin: 0,
                  whiteSpace: 'pre-wrap',
                  wordBreak: 'break-word',
                  color: '#e5e7eb',
                  maxHeight: 300,
                  overflowY: 'auto'
                }}>
                  {JSON.stringify(msg.data, null, 2)}
                </pre>
              </div>
            ))
          )}
          <div ref={logEndRef} />
        </div>
      )}
    </div>
  );
}

function getTypeColor(type: string): string {
  const colors: Record<string, string> = {
    'toolCall': '#10b981',     // green
    'message': '#3b82f6',      // blue
    'audio': '#8b5cf6',        // purple
    'error': '#ef4444',        // red
    'open': '#10b981',         // green
    'close': '#6b7280',        // gray
  };
  return colors[type] || '#6b7280';
}
