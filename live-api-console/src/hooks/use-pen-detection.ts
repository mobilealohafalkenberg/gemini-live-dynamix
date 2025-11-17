/**
 * Custom hook for pen detection with Python bridge
 */

import { useEffect, useCallback, useState } from 'react';
import { LiveServerToolCall } from '@google/genai';

export interface PenDetectionState {
  isHolding: boolean | null;
  lastUpdate: Date | null;
  detectionCount: number;
  pythonConnected: boolean;
}

export function usePenDetection() {
  const [state, setState] = useState<PenDetectionState>({
    isHolding: null,
    lastUpdate: null,
    detectionCount: 0,
    pythonConnected: false,
  });

  // Check Python server connection
  useEffect(() => {
    const checkConnection = async () => {
      try {
        const response = await fetch('http://localhost:8081/status');
        if (response.ok) {
          setState(prev => ({ ...prev, pythonConnected: true }));
        }
      } catch {
        setState(prev => ({ ...prev, pythonConnected: false }));
      }
    };

    checkConnection();
    const interval = setInterval(checkConnection, 5000);
    return () => clearInterval(interval);
  }, []);

  const handleToolCall = useCallback(async (toolCall: LiveServerToolCall) => {
    console.log('Tool call received:', toolCall);
    
    const functionCalls = toolCall.functionCalls || [];
    
    for (const call of functionCalls) {
      if (call.name === 'holding_pen') {
        const isHolding = Boolean(call.args?.is_holding);
        
        // Update local state
        setState(prev => ({
          ...prev,
          isHolding: isHolding,
          lastUpdate: new Date(),
          detectionCount: prev.detectionCount + 1,
        }));

        // Send to Python server if connected
        if (state.pythonConnected) {
          try {
            const response = await fetch('http://localhost:8081/tool-call', {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify({
                name: call.name,
                args: call.args,
                id: call.id,
              }),
            });

            if (!response.ok) {
              console.error('Failed to send to Python server');
            }
          } catch (error) {
            console.error('Error sending to Python:', error);
          }
        }

        // Visual feedback
        const emoji = isHolding ? '✅' : '❌';
        const status = isHolding ? 'PEN DETECTED' : 'NO PEN';
        console.log(`${emoji} ${status}`);
        
        // Update page title for easy visibility
        document.title = `${emoji} ${status} | Live API Console`;
      }
    }
  }, [state.pythonConnected]);

  return {
    state,
    handleToolCall,
  };
}