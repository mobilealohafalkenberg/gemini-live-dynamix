import React from 'react';

interface ModeSelectorProps {
  onSelectMode: (mode: 'glasses' | 'robot' | 'mac') => void;
}

export function ModeSelector({ onSelectMode }: ModeSelectorProps) {
  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      justifyContent: 'center',
      height: '100vh',
      background: '#1a1a1a',
      color: 'white'
    }}>
      <h1 style={{ marginBottom: 40, fontSize: 32 }}>Select Detection Mode</h1>
      
      <div style={{ display: 'flex', gap: 30 }}>
        {/* Glasses Detection Button */}
        <button
          onClick={() => onSelectMode('glasses')}
          style={{
            padding: '40px',
            fontSize: 20,
            background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
            color: 'white',
            border: 'none',
            borderRadius: 12,
            cursor: 'pointer',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            gap: 15,
            minWidth: 200,
            transition: 'transform 0.2s',
          }}
          onMouseEnter={(e) => e.currentTarget.style.transform = 'scale(1.05)'}
          onMouseLeave={(e) => e.currentTarget.style.transform = 'scale(1)'}
        >
          <span style={{ fontSize: 48 }}>👓</span>
          <span>Glasses Detection</span>
          <span style={{ fontSize: 14, opacity: 0.9 }}>
            Detect if wearing glasses
          </span>
        </button>

        {/* Robot Control Button */}
        <button
          onClick={() => onSelectMode('robot')}
          style={{
            padding: '40px',
            fontSize: 20,
            background: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
            color: 'white',
            border: 'none',
            borderRadius: 12,
            cursor: 'pointer',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            gap: 15,
            minWidth: 200,
            transition: 'transform 0.2s',
          }}
          onMouseEnter={(e) => e.currentTarget.style.transform = 'scale(1.05)'}
          onMouseLeave={(e) => e.currentTarget.style.transform = 'scale(1)'}
        >
          <span style={{ fontSize: 48 }}>🤖</span>
          <span>ALOHA Robot Control</span>
          <span style={{ fontSize: 14, opacity: 0.9 }}>
            Control robot actions
          </span>
        </button>

        {/* Mac Control Button */}
        <button
          onClick={() => onSelectMode('mac')}
          style={{
            padding: '40px',
            fontSize: 20,
            background: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
            color: 'white',
            border: 'none',
            borderRadius: 12,
            cursor: 'pointer',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            gap: 15,
            minWidth: 200,
            transition: 'transform 0.2s',
          }}
          onMouseEnter={(e) => e.currentTarget.style.transform = 'scale(1.05)'}
          onMouseLeave={(e) => e.currentTarget.style.transform = 'scale(1)'}
        >
          <span style={{ fontSize: 48 }}>🖥️</span>
          <span>Mac Control</span>
          <span style={{ fontSize: 14, opacity: 0.9 }}>
            Control your Mac
          </span>
        </button>
      </div>

      <p style={{ marginTop: 40, opacity: 0.7 }}>
        Choose which mode you want to use with Gemini Live API
      </p>
    </div>
  );
}