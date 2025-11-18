# Simplified Robotics ER Setup for Debugging

**Purpose:** A simplified text-based interface for testing Gemini Robotics ER 1.5 without the complexity of WebSocket streaming and voice input.

**Documentation:** https://ai.google.dev/gemini-api/docs/robotics-overview

---

## Problem Statement

The current system uses Gemini Live API with:
- WebSocket streaming
- Voice input
- Fire-and-forget tool responses
- Async event handling

This makes debugging difficult when:
- Coordinates don't match expected positions
- It's unclear if the issue is in vision, kinematics, or rendering
- Responses are hard to reproduce

---

## Proposed Solution: Simplified Text-Based Flow

```
USER INPUT (Text)
    │
    │ "Pick up red cube and place on bowl"
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 1. REQUEST BUILDER (Frontend)                                   │
│    - Captures text input                                        │
│    - Gets current simulation state (robot pose, objects)        │
│    - Captures camera frames (base64 JPEG)                       │
│    - Builds single REST API request                             │
└─────────────────────────────────────────────────────────────────┘
    │
    │ POST Request with:
    │ {
    │   prompt: "Pick up red cube...",
    │   images: [gripper_cam_b64, top_cam_b64],
    │   robot_state: { joints, gripper, ee_position },
    │   scene_objects: [{ name: "red_cube", position: [...] }],
    │   tools: [ /* tool definitions */ ]
    │ }
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. SIMPLE PYTHON BRIDGE (New, port 8082)                        │
│    - Single endpoint: POST /robotics-er-request                 │
│    - No hardware controllers needed                             │
│    - Pure API wrapper                                           │
└─────────────────────────────────────────────────────────────────┘
    │
    │ Formatted request
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. GEMINI ROBOTICS ER 1.5 (REST API)                            │
│    Documentation: https://ai.google.dev/gemini-api/docs/        │
│                   robotics-overview                             │
│                                                                  │
│    - Analyzes images (object detection)                         │
│    - Plans trajectory                                           │
│    - Returns function calls with coordinates                    │
└─────────────────────────────────────────────────────────────────┘
    │
    │ Response:
    │ {
    │   object_detected: { name: "red_cube", pos: [x,y,z] },
    │   target_detected: { name: "bowl", pos: [x,y,z] },
    │   reasoning: "...",
    │   function_calls: [
    │     { name: "move_arm_trajectory", args: { ... } },
    │     { name: "control_gripper", args: { ... } }
    │   ]
    │ }
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. RESPONSE PARSER (Frontend)                                   │
│    - Logs full response for debugging                           │
│    - Extracts function calls                                    │
│    - Validates coordinates                                      │
│    - Queues for execution                                       │
└─────────────────────────────────────────────────────────────────┘
    │
    │ Parsed tool calls
    ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. LOCAL EXECUTOR (Frontend)                                    │
│    - Same RobotController as current system                     │
│    - Execute tool calls sequentially                            │
│    - Log each step                                              │
│    - Display results in UI                                      │
└─────────────────────────────────────────────────────────────────┘
    │
    │ Visual feedback
    ▼
THREE.JS SIMULATION (Same as current)
```

---

## Implementation Plan

### File Structure

```
virtual-robot-arm/
├── src/
│   ├── components/
│   │   ├── SimpleERControl.tsx          ← NEW: Text input + ER testing UI
│   │   └── DebugCoordinateOverlay.tsx   ← NEW: Shows coordinates in 3D
│   │
│   └── lib/
│       └── robotics-er-client.ts         ← NEW: REST API client for ER

gemini-live/gemini-live-api-control/bridges/
└── bridge_simple_er.py                   ← NEW: Simple REST bridge (port 8082)
```

---

## Component 1: Frontend Request Builder

**File:** `virtual-robot-arm/src/lib/robotics-er-client.ts`

```typescript
/**
 * Simple REST client for Gemini Robotics ER
 * Documentation: https://ai.google.dev/gemini-api/docs/robotics-overview
 */

import { RobotState, ROBOT_SPECS } from '../types/robot';

export interface SceneObject {
  name: string;
  position: [number, number, number];
  color?: string;
  type?: string;
}

export interface ERRequest {
  prompt: string;
  images: string[];  // base64 JPEG
  robotState: RobotState;
  sceneObjects: SceneObject[];
}

export interface ERResponse {
  text?: string;
  reasoning?: string;
  object_detected?: {
    name: string;
    pos: [number, number, number];
    confidence: number;
  };
  target_detected?: {
    name: string;
    pos: [number, number, number];
    confidence: number;
  };
  function_calls: Array<{
    name: string;
    args: any;
  }>;
  error?: string;
}

export class RoboticsERClient {
  private bridgeUrl: string;

  constructor(bridgeUrl: string = 'http://localhost:8082') {
    this.bridgeUrl = bridgeUrl;
  }

  async sendRequest(params: ERRequest): Promise<ERResponse> {
    console.log('[RoboticsER] Sending request:', {
      prompt: params.prompt,
      images_count: params.images.length,
      robot_joints: params.robotState.joints,
      scene_objects: params.sceneObjects.length
    });

    // Build request payload
    const request = {
      prompt: params.prompt,
      images: params.images,
      context: {
        robot_specs: {
          model: 'ViperX 300s',
          dof: 6,
          joint_limits: ROBOT_SPECS.jointLimits,
          link_lengths: ROBOT_SPECS.linkLengths,
        },
        current_state: {
          joints: params.robotState.joints,
          joints_degrees: Object.values(params.robotState.joints).map(
            rad => (rad * 180) / Math.PI
          ),
          gripper_state: params.robotState.gripperState,
          ee_position: params.robotState.endEffectorPosition,
        },
        scene_objects: params.sceneObjects,
        workspace_bounds: {
          x: [-0.8, 0.0],
          y: [-0.4, 0.4],
          z: [0.1, 0.6]
        },
        robot_base_position: [-0.469, -0.019, 0.02]
      },
      tools: this.getToolDefinitions()
    };

    console.log('[RoboticsER] Request payload:', JSON.stringify(request, null, 2));

    try {
      // Send to simple Python bridge
      const response = await fetch(`${this.bridgeUrl}/robotics-er-request`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request)
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const result = await response.json();
      console.log('[RoboticsER] Response received:', result);

      return result;
    } catch (error) {
      console.error('[RoboticsER] Request failed:', error);
      return {
        function_calls: [],
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  }

  private getToolDefinitions() {
    // Same tool definitions as Live API
    return [
      {
        name: 'move_arm',
        description: 'Move robot arm to target position or pose',
        parameters: {
          type: 'object',
          properties: {
            pose: {
              type: 'string',
              description: 'Named pose: home, sleep, or ready',
              enum: ['home', 'sleep', 'ready']
            },
            position: {
              type: 'array',
              description: 'Cartesian position [x,y,z] in meters',
              items: { type: 'number' }
            },
            moving_time: {
              type: 'number',
              description: 'Time to complete movement in seconds'
            }
          }
        }
      },
      {
        name: 'move_arm_trajectory',
        description: 'Execute multi-waypoint trajectory with gripper coordination',
        parameters: {
          type: 'object',
          properties: {
            trajectory: {
              type: 'array',
              description: 'Array of waypoints',
              items: {
                type: 'object',
                properties: {
                  point: {
                    type: 'array',
                    description: 'Position [x,y,z] in meters',
                    items: { type: 'number' }
                  },
                  label: {
                    type: 'string',
                    description: 'Descriptive label for waypoint'
                  },
                  gripper_action: {
                    type: 'string',
                    description: 'Gripper action at this waypoint',
                    enum: ['open', 'close', 'maintain']
                  }
                },
                required: ['point']
              }
            },
            speed: {
              type: 'string',
              description: 'Trajectory speed',
              enum: ['slow', 'medium', 'fast']
            }
          },
          required: ['trajectory']
        }
      },
      {
        name: 'control_gripper',
        description: 'Open or close the robot gripper',
        parameters: {
          type: 'object',
          properties: {
            action: {
              type: 'string',
              description: 'Gripper action',
              enum: ['open', 'close']
            }
          },
          required: ['action']
        }
      },
      {
        name: 'get_arm_status',
        description: 'Get current arm state and position',
        parameters: {
          type: 'object',
          properties: {}
        }
      },
      {
        name: 'get_gripper_status',
        description: 'Get current gripper state',
        parameters: {
          type: 'object',
          properties: {}
        }
      }
    ];
  }
}
```

---

## Component 2: Simple Python Bridge

**File:** `gemini-live/gemini-live-api-control/bridges/bridge_simple_er.py`

```python
#!/usr/bin/env python3
"""
Simple REST bridge for Gemini Robotics ER testing.
Documentation: https://ai.google.dev/gemini-api/docs/robotics-overview

This bridge provides a simple synchronous interface to test Robotics ER
without the complexity of WebSocket streaming.

Usage:
    python3 bridges/bridge_simple_er.py
"""

import os
import sys
import json
import asyncio
from pathlib import Path
from aiohttp import web
from aiohttp_cors import setup, ResourceOptions

# Import Google Generative AI (new SDK)
try:
    from google import genai
    from google.genai import types
except ImportError as e:
    print("=" * 60)
    print("ERROR: google-genai SDK not installed")
    print("=" * 60)
    print()
    print("This bridge requires the google-genai package.")
    print("Install it with:")
    print()
    print("    pip install google-genai")
    print()
    print("Documentation: https://googleapis.github.io/python-genai/")
    print("=" * 60)
    raise ImportError("google-genai package is required") from e


async def handle_er_request(request: web.Request) -> web.Response:
    """
    Handle Robotics ER request.

    Request format:
    {
        "prompt": "Pick up red cube and place on bowl",
        "images": ["<base64_jpeg>", "<base64_jpeg>"],
        "context": {
            "robot_specs": {...},
            "current_state": {...},
            "scene_objects": [...],
            "workspace_bounds": {...}
        },
        "tools": [...]
    }
    """

    try:
        data = await request.json()

        prompt = data.get('prompt', '')
        images = data.get('images', [])
        context = data.get('context', {})
        tools = data.get('tools', [])

        print(f"\n{'=' * 60}")
        print(f"[Simple ER Bridge] Request received")
        print(f"{'=' * 60}")
        print(f"Prompt: {prompt}")
        print(f"Images: {len(images)} frames")
        print(f"Scene objects: {len(context.get('scene_objects', []))}")
        print(f"Tools: {len(tools)}")

        # Configure Gemini
        api_key = os.getenv('GEMINI_API_KEY')
        if not api_key:
            return web.json_response({
                'success': False,
                'error': 'GEMINI_API_KEY environment variable not set'
            }, status=500)

        genai.configure(api_key=api_key)

        # Build context prompt
        context_prompt = f"""
TASK: {prompt}

ROBOT CONTEXT:
- Model: {context.get('robot_specs', {}).get('model', 'Unknown')}
- Current joints (degrees): {context.get('current_state', {}).get('joints_degrees', [])}
- Gripper state: {context.get('current_state', {}).get('gripper_state', {})}
- End-effector position: {context.get('current_state', {}).get('ee_position', {})}
- Robot base in world: {context.get('robot_base_position', [])}

SCENE OBJECTS:
{json.dumps(context.get('scene_objects', []), indent=2)}

WORKSPACE BOUNDS (in meters, relative to robot base):
- X range: {context.get('workspace_bounds', {}).get('x', [])}
- Y range: {context.get('workspace_bounds', {}).get('y', [])}
- Z range: {context.get('workspace_bounds', {}).get('z', [])}

INSTRUCTIONS:
1. Analyze the provided camera images (gripper view + overhead view)
2. Detect objects mentioned in the task
3. Plan a trajectory to accomplish the task
4. Use function calls to execute the plan
5. Ensure all coordinates are within workspace bounds
6. Provide reasoning for your decisions
"""

        print(f"\nContext prompt:\n{context_prompt}\n")

        # Create model with tools
        # See: https://ai.google.dev/gemini-api/docs/robotics-overview
        model = genai.GenerativeModel(
            'gemini-2.0-flash-exp',  # or gemini-1.5-pro for more reasoning
            tools=tools
        )

        # Build content parts
        content_parts = [context_prompt]

        # Add images if provided
        for i, img_b64 in enumerate(images):
            content_parts.append({
                'mime_type': 'image/jpeg',
                'data': img_b64
            })
            print(f"Added image {i+1}/{len(images)}")

        # Generate content
        print("\nSending to Gemini Robotics ER...")
        response = model.generate_content(content_parts)

        print(f"\n{'=' * 60}")
        print(f"[Simple ER Bridge] Response received")
        print(f"{'=' * 60}")

        # Parse response
        result = {
            'success': True,
            'text': None,
            'function_calls': [],
            'reasoning': None
        }

        # Extract text response
        if response.text:
            result['text'] = response.text
            result['reasoning'] = response.text
            print(f"Text response: {response.text}")

        # Extract function calls
        if response.candidates and len(response.candidates) > 0:
            candidate = response.candidates[0]
            if candidate.content and candidate.content.parts:
                for part in candidate.content.parts:
                    if hasattr(part, 'function_call'):
                        fc = part.function_call
                        function_call = {
                            'name': fc.name,
                            'args': dict(fc.args)
                        }
                        result['function_calls'].append(function_call)
                        print(f"\nFunction call: {fc.name}")
                        print(f"Arguments: {json.dumps(dict(fc.args), indent=2)}")

        print(f"\nTotal function calls: {len(result['function_calls'])}")
        print(f"{'=' * 60}\n")

        return web.json_response(result)

    except Exception as e:
        print(f"\n[Simple ER Bridge] ERROR: {e}")
        import traceback
        traceback.print_exc()

        return web.json_response({
            'success': False,
            'error': str(e)
        }, status=500)


async def handle_status(request: web.Request) -> web.Response:
    """Status endpoint"""
    return web.json_response({
        'bridge': 'simple_er',
        'sdk': 'google-genai',
        'status': 'running',
        'genai_available': True,  # If we got here, SDK is available
        'api_key_set': bool(os.getenv('GEMINI_API_KEY'))
    })


def make_app() -> web.Application:
    """Create the application"""
    app = web.Application()

    # Setup CORS for browser access
    cors = setup(app, defaults={
        '*': ResourceOptions(
            allow_credentials=True,
            expose_headers='*',
            allow_headers='*',
            allow_methods='*'
        )
    })

    # Add routes
    app.router.add_post('/robotics-er-request', handle_er_request)
    app.router.add_get('/status', handle_status)

    # Add CORS to routes
    for route in list(app.router.routes()):
        cors.add(route)

    return app


if __name__ == '__main__':
    print("=" * 60)
    print("Simple Robotics ER Bridge")
    print("=" * 60)
    print()
    print("Documentation: https://ai.google.dev/gemini-api/docs/robotics-overview")
    print()
    print("This bridge provides a simple REST API for testing Gemini")
    print("Robotics ER without WebSocket complexity.")
    print()
    print("Starting server on http://localhost:8082")
    print("Endpoints:")
    print("  - POST /robotics-er-request")
    print("  - GET /status")
    print()

    if not os.getenv('GEMINI_API_KEY'):
        print("WARNING: GEMINI_API_KEY environment variable not set!")
        print("Set it in your shell or .env file")
        print()

    # Run the server
    web.run_app(make_app(), host='0.0.0.0', port=8082)
```

---

## Component 3: UI Component

**File:** `virtual-robot-arm/src/components/SimpleERControl.tsx`

```typescript
/**
 * Simple ER Control - Text-based interface for debugging
 * Uses synchronous REST API instead of WebSocket streaming
 */

import { useState, useRef } from 'react';
import { RobotController } from '../lib/robot-controller';
import { RoboticsERClient, SceneObject, ERResponse } from '../lib/robotics-er-client';

interface SimpleERControlProps {
  robotController: RobotController;
  cameraViews: Array<{ name: string; imageData: string }>;
  sceneObjects: SceneObject[];
}

export function SimpleERControl({
  robotController,
  cameraViews,
  sceneObjects
}: SimpleERControlProps) {
  const [textInput, setTextInput] = useState('');
  const [erResponse, setERResponse] = useState<ERResponse | null>(null);
  const [executing, setExecuting] = useState(false);
  const [executionLog, setExecutionLog] = useState<string[]>([]);
  const [showDebug, setShowDebug] = useState(true);

  const erClient = useRef(new RoboticsERClient('http://localhost:8082'));

  const addLog = (message: string) => {
    const timestamp = new Date().toLocaleTimeString();
    setExecutionLog(prev => [...prev, `[${timestamp}] ${message}`]);
    console.log(`[SimpleER] ${message}`);
  };

  const executeToolCall = async (name: string, args: any) => {
    addLog(`Executing: ${name}`);
    console.log(`[SimpleER] Tool: ${name}`, args);

    try {
      switch (name) {
        case 'move_arm':
          if (args.pose) {
            robotController.moveToNamedPose(args.pose, args.moving_time || 1.5);
            addLog(`Moving to ${args.pose} pose`);
          } else if (args.position) {
            const success = robotController.moveToPosition(args.position, args.moving_time || 1.5);
            if (success) {
              addLog(`Moving to position [${args.position.join(', ')}]`);
            } else {
              addLog(`❌ Failed to reach position [${args.position.join(', ')}]`);
            }
          }
          break;

        case 'move_arm_trajectory':
          addLog(`Executing trajectory with ${args.trajectory?.length || 0} waypoints`);
          const success = await robotController.executeTrajectory(
            args.trajectory,
            args.speed || 'medium'
          );
          if (success) {
            addLog(`✓ Trajectory completed`);
          } else {
            addLog(`❌ Trajectory failed`);
          }
          break;

        case 'control_gripper':
          if (args.action === 'open') {
            robotController.openGripper();
            addLog('Opening gripper');
          } else if (args.action === 'close') {
            robotController.closeGripper();
            addLog('Closing gripper');
          }
          break;

        case 'get_arm_status':
          const armStatus = robotController.getArmStatus();
          addLog(`Arm status: ${armStatus.state}`);
          console.log('Arm status:', armStatus);
          break;

        case 'get_gripper_status':
          const gripperStatus = robotController.getGripperStatus();
          addLog(`Gripper: ${gripperStatus.state}`);
          console.log('Gripper status:', gripperStatus);
          break;

        default:
          addLog(`⚠️ Unknown tool: ${name}`);
      }
    } catch (error) {
      addLog(`❌ Error: ${error}`);
      console.error('Tool execution error:', error);
    }
  };

  const handleSubmit = async () => {
    if (!textInput.trim()) return;

    addLog(`User: "${textInput}"`);
    setExecutionLog([]);
    setERResponse(null);
    setExecuting(true);

    try {
      // Send request to ER
      const response = await erClient.current.sendRequest({
        prompt: textInput,
        images: cameraViews.map(v => v.imageData),
        robotState: robotController.getState(),
        sceneObjects: sceneObjects
      });

      setERResponse(response);

      if (response.error) {
        addLog(`❌ ER Error: ${response.error}`);
        setExecuting(false);
        return;
      }

      if (response.reasoning) {
        addLog(`ER Reasoning: ${response.reasoning}`);
      }

      // Execute function calls sequentially
      if (response.function_calls && response.function_calls.length > 0) {
        addLog(`Executing ${response.function_calls.length} tool calls...`);

        for (let i = 0; i < response.function_calls.length; i++) {
          const call = response.function_calls[i];
          addLog(`[${i + 1}/${response.function_calls.length}] ${call.name}`);

          await executeToolCall(call.name, call.args);

          // Wait between calls
          if (i < response.function_calls.length - 1) {
            await new Promise(r => setTimeout(r, 1000));
          }
        }

        addLog('✓ All tool calls completed');
      } else {
        addLog('No tool calls in response');
      }

    } catch (error) {
      addLog(`❌ Request failed: ${error}`);
      console.error('ER request error:', error);
    } finally {
      setExecuting(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <div style={{
      position: 'absolute',
      top: 20,
      right: 20,
      width: 400,
      maxHeight: 'calc(100vh - 40px)',
      background: 'rgba(0, 0, 0, 0.9)',
      color: 'white',
      padding: 20,
      borderRadius: 8,
      overflow: 'auto',
      fontFamily: 'monospace',
      fontSize: 12
    }}>
      <h3 style={{ margin: '0 0 16px 0', fontSize: 16 }}>
        Simple ER Testing
      </h3>

      {/* Status */}
      <div style={{ marginBottom: 16, fontSize: 11, opacity: 0.7 }}>
        Bridge: http://localhost:8082
        <br />
        Camera frames: {cameraViews.length}
        <br />
        Scene objects: {sceneObjects.length}
      </div>

      {/* Text input */}
      <textarea
        value={textInput}
        onChange={e => setTextInput(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder="Enter command (e.g., 'Pick up red cube and place on bowl')"
        disabled={executing}
        style={{
          width: '100%',
          height: 80,
          padding: 8,
          background: 'rgba(255, 255, 255, 0.1)',
          border: '1px solid #555',
          borderRadius: 4,
          color: 'white',
          fontFamily: 'monospace',
          fontSize: 12,
          resize: 'vertical'
        }}
      />

      {/* Submit button */}
      <button
        onClick={handleSubmit}
        disabled={executing || !textInput.trim()}
        style={{
          width: '100%',
          marginTop: 8,
          padding: 12,
          background: executing ? '#666' : '#4CAF50',
          color: 'white',
          border: 'none',
          borderRadius: 4,
          cursor: executing ? 'not-allowed' : 'pointer',
          fontSize: 14,
          fontWeight: 'bold'
        }}
      >
        {executing ? 'Processing...' : 'Send to Robotics ER'}
      </button>

      {/* Quick test commands */}
      <div style={{ marginTop: 16 }}>
        <div style={{ fontSize: 11, opacity: 0.7, marginBottom: 8 }}>Quick commands:</div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
          <button
            onClick={() => setTextInput('Pick up the green cube')}
            disabled={executing}
            style={{ padding: 6, fontSize: 11, background: '#2196F3', color: 'white', border: 'none', borderRadius: 3, cursor: 'pointer' }}
          >
            Pick green cube
          </button>
          <button
            onClick={() => setTextInput('Move to home position')}
            disabled={executing}
            style={{ padding: 6, fontSize: 11, background: '#2196F3', color: 'white', border: 'none', borderRadius: 3, cursor: 'pointer' }}
          >
            Go home
          </button>
        </div>
      </div>

      {/* Response display */}
      {erResponse && (
        <div style={{ marginTop: 16 }}>
          <div style={{
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            marginBottom: 8
          }}>
            <strong>ER Response:</strong>
            <button
              onClick={() => setShowDebug(!showDebug)}
              style={{
                padding: '4px 8px',
                fontSize: 10,
                background: '#555',
                color: 'white',
                border: 'none',
                borderRadius: 3,
                cursor: 'pointer'
              }}
            >
              {showDebug ? 'Hide' : 'Show'} Debug
            </button>
          </div>

          {/* Detected objects */}
          {erResponse.object_detected && (
            <div style={{
              padding: 8,
              background: 'rgba(76, 175, 80, 0.2)',
              borderRadius: 4,
              marginBottom: 8
            }}>
              <div style={{ fontSize: 11, opacity: 0.7 }}>Object detected:</div>
              <div>{erResponse.object_detected.name}</div>
              <div style={{ fontSize: 10, opacity: 0.7 }}>
                Position: [{erResponse.object_detected.pos.map(v => v.toFixed(3)).join(', ')}]
              </div>
              <div style={{ fontSize: 10, opacity: 0.7 }}>
                Confidence: {(erResponse.object_detected.confidence * 100).toFixed(1)}%
              </div>
            </div>
          )}

          {/* Function calls */}
          <div style={{ marginBottom: 8 }}>
            <strong>Tool Calls ({erResponse.function_calls?.length || 0}):</strong>
          </div>
          {erResponse.function_calls?.map((call, i) => (
            <div
              key={i}
              style={{
                padding: 8,
                background: 'rgba(33, 150, 243, 0.2)',
                borderRadius: 4,
                marginBottom: 4,
                fontSize: 11
              }}
            >
              <div><strong>{i + 1}. {call.name}</strong></div>
              <pre style={{
                margin: '4px 0 0 0',
                fontSize: 10,
                opacity: 0.8,
                whiteSpace: 'pre-wrap',
                wordBreak: 'break-word'
              }}>
                {JSON.stringify(call.args, null, 2)}
              </pre>
            </div>
          ))}

          {/* Debug info */}
          {showDebug && (
            <details style={{ marginTop: 8 }}>
              <summary style={{ cursor: 'pointer', fontSize: 11, opacity: 0.7 }}>
                Full Response JSON
              </summary>
              <pre style={{
                marginTop: 8,
                padding: 8,
                background: 'rgba(255, 255, 255, 0.05)',
                borderRadius: 4,
                fontSize: 10,
                overflow: 'auto',
                maxHeight: 200
              }}>
                {JSON.stringify(erResponse, null, 2)}
              </pre>
            </details>
          )}
        </div>
      )}

      {/* Execution log */}
      {executionLog.length > 0 && (
        <div style={{ marginTop: 16 }}>
          <strong>Execution Log:</strong>
          <div style={{
            marginTop: 8,
            padding: 8,
            background: 'rgba(0, 0, 0, 0.5)',
            borderRadius: 4,
            maxHeight: 200,
            overflow: 'auto',
            fontSize: 10
          }}>
            {executionLog.map((log, i) => (
              <div key={i} style={{ marginBottom: 2 }}>{log}</div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}
```

---

## Component 4: Integration with Main App

**File:** `virtual-robot-arm/src/components/IntegratedRobotControl.tsx` (modifications)

Add toggle between normal mode and Simple ER mode:

```typescript
// Add to IntegratedRobotControl component
import { SimpleERControl } from './SimpleERControl';

export function IntegratedRobotControl() {
  // ... existing state ...
  const [useSimpleER, setUseSimpleER] = useState(false);

  // ... existing code ...

  return (
    <div style={{ position: 'relative', width: '100%', height: '100vh' }}>
      {/* 3D Visualization */}
      {robotState && (
        <Scene
          robotState={robotState}
          onCameraUpdate={handleCameraUpdate}
          targetPosition={targetPosition}
          topCameraAdjustments={topCameraAdjustments}
        />
      )}

      {/* Mode toggle */}
      <button
        onClick={() => setUseSimpleER(!useSimpleER)}
        style={{
          position: 'absolute',
          top: 20,
          left: 20,
          padding: '8px 16px',
          background: useSimpleER ? '#FF9800' : '#4CAF50',
          color: 'white',
          border: 'none',
          borderRadius: 4,
          cursor: 'pointer',
          zIndex: 1000
        }}
      >
        {useSimpleER ? '🧪 Simple ER Mode' : '🎤 Voice Mode'}
      </button>

      {/* Control UI - conditional */}
      {useSimpleER ? (
        <SimpleERControl
          robotController={robotController}
          cameraViews={cameraViews}
          sceneObjects={[
            { name: 'green_cube', position: [-0.15, 0.15, 0.14], color: 'green' },
            { name: 'blue_cube', position: [-0.2, -0.1, 0.14], color: 'blue' }
          ]}
        />
      ) : (
        <VirtualRobotControl
          robotController={robotController}
          onRobotStateChange={setRobotState}
          cameraViews={cameraViews}
          onTargetPositionChange={handleTargetPositionChange}
          onCameraAdjustment={handleCameraAdjustment}
        />
      )}

      {/* Manual controls */}
      <ManualJointControls
        robotController={robotController}
        visible={showManualControls}
        onToggle={() => setShowManualControls(!showManualControls)}
      />
    </div>
  );
}
```

---

## Debugging Workflow

### 1. **Start the Bridge**
```bash
cd gemini-live/gemini-live-api-control

# Set API key
export GEMINI_API_KEY="your-api-key-here"

# Start bridge
python3 bridges/bridge_simple_er.py

# Should see:
# Starting server on http://localhost:8082
```

### 2. **Start the Virtual Robot**
```bash
cd virtual-robot-arm

# Start dev server
npm start

# Opens http://localhost:3000
```

### 3. **Toggle to Simple ER Mode**
- Click "🧪 Simple ER Mode" button in top-left
- Simple text interface appears on right side

### 4. **Test with Known Objects**
```
Input: "Pick up the green cube"

Expected flow:
1. Captures 2 camera frames (gripper + top)
2. Sends to bridge with scene context
3. ER analyzes images
4. ER detects green cube at [-0.15, 0.15, 0.14]
5. ER plans trajectory
6. ER returns function calls
7. UI executes trajectory
8. Robot moves to cube
```

### 5. **Debug Coordinate Issues**

**Check ER Detection:**
```javascript
// In browser console after sending request:
console.log('ER detected cube at:', erResponse.object_detected.pos);
console.log('Actual cube position:', [-0.15, 0.15, 0.14]);

// Calculate error
const detected = erResponse.object_detected.pos;
const actual = [-0.15, 0.15, 0.14];
const error = Math.sqrt(
  Math.pow(detected[0] - actual[0], 2) +
  Math.pow(detected[1] - actual[1], 2) +
  Math.pow(detected[2] - actual[2], 2)
);
console.log('Detection error:', error * 1000, 'mm');
```

**Check IK Accuracy:**
```javascript
// After ER returns trajectory
const targetPos = erResponse.function_calls[0].args.trajectory[0].point;
console.log('ER wants to go to:', targetPos);

// Execute movement
// ... movement happens ...

// Check where robot actually went
const finalPos = robotController.getState().endEffectorPosition;
console.log('Robot reached:', [
  finalPos.x,
  finalPos.y,
  finalPos.z
]);

// Calculate IK error
const ikError = Math.sqrt(
  Math.pow(targetPos[0] - finalPos.x, 2) +
  Math.pow(targetPos[1] - finalPos.y, 2) +
  Math.pow(targetPos[2] - finalPos.z, 2)
);
console.log('IK error:', ikError * 1000, 'mm');
```

---

## Key Advantages for Debugging

### 1. **Complete Visibility**
Every step is logged and visible:
- Request payload (prompt, images, context)
- ER response (detection, reasoning, tool calls)
- Execution steps (each movement)
- Final positions (where robot ended up)

### 2. **Reproducibility**
Same text input = same request = reproducible results

### 3. **Coordinate Validation**
Easy to compare:
- Known object positions (ground truth)
- ER detected positions
- IK calculated positions
- Final robot positions

### 4. **Step-by-Step Execution**
Can pause, inspect, and continue at each stage

---

## Comparison: Simple ER vs Live API

| Feature | Simple ER (Debugging) | Live API (Production) |
|---------|----------------------|----------------------|
| **Input** | Text | Voice |
| **API** | REST (synchronous) | WebSocket (streaming) |
| **Response Time** | 2-5 seconds | 500-2000ms |
| **Debugging** | ✅ Easy (full logs) | ❌ Hard (async events) |
| **Reproducibility** | ✅ Perfect | ❌ Voice varies |
| **Use Case** | Development/Testing | End-user experience |
| **Complexity** | Low (100 lines) | High (1000+ lines) |
| **Documentation** | https://ai.google.dev/gemini-api/docs/robotics-overview | Live API docs |

---

## Environment Variables

Add to `virtual-robot-arm/.env`:
```bash
# Existing
REACT_APP_GEMINI_API_KEY=your-api-key-here

# New (optional - for bridge mode)
REACT_APP_SIMPLE_ER_BRIDGE=http://localhost:8082
```

For Python bridge, set in shell:
```bash
export GEMINI_API_KEY=your-api-key-here
```

---

## Testing Checklist

- [ ] Bridge starts without errors
- [ ] Status endpoint returns correct info: `curl http://localhost:8082/status`
- [ ] UI loads with Simple ER mode toggle
- [ ] Text input sends request to bridge
- [ ] Bridge logs show request received
- [ ] Gemini returns function calls
- [ ] UI displays response correctly
- [ ] Tool calls execute in sequence
- [ ] Robot moves to expected positions
- [ ] Coordinates match between ER, IK, and visual

---

## Common Issues

### Bridge fails to start
```
ERROR: google-genai SDK not installed
Solution: pip install google-genai
```

### No API key
```
WARNING: GEMINI_API_KEY not set
Solution: export GEMINI_API_KEY=your-key
```

### CORS errors
```
Bridge already has CORS enabled. Check browser console for details.
```

### Empty function calls
```
Check if tools are properly defined in request.
ER should see tool definitions in the request payload.
```

---

## Next Steps

Once debugging is complete with Simple ER, findings can be applied back to the Live API system:
1. Fix coordinate transformations
2. Validate IK accuracy
3. Improve trajectory planning
4. Then re-enable voice control with confidence

---

## References

- **Gemini Robotics Overview:** https://ai.google.dev/gemini-api/docs/robotics-overview
- **Function Calling Guide:** https://ai.google.dev/gemini-api/docs/function-calling
- **Python SDK:** https://github.com/googleapis/python-genai
- **Google Gemini Cookbook** https://github.com/google-gemini/cookbook
- **Current System:** See `flow_debug.md` for full architecture