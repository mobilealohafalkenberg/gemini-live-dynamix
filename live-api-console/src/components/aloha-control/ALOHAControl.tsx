import { useEffect, useState } from 'react';
import { useLiveAPIContext } from '../../contexts/LiveAPIContext';
import { FunctionDeclaration, Type } from '@google/genai';
import { ApiResponseViewer } from '../api-response-viewer/ApiResponseViewer';

const ROBOT_ENDPOINT = process.env.REACT_APP_ROBOT_ENDPOINT || 'http://localhost:8081';

const toolControlGripper: FunctionDeclaration = {
  name: 'control_gripper',
  description: 'Open or close the robot gripper',
  parameters: {
    type: Type.OBJECT,
    properties: {
      action: { type: Type.STRING, description: 'open or close' },
    },
    required: ['action'],
  },
};

const toolGetGripperStatus: FunctionDeclaration = {
  name: 'get_gripper_status',
  description: 'Get current gripper state and position',
  parameters: { type: Type.OBJECT, properties: {}, required: [] },
};

const toolMoveArm: FunctionDeclaration = {
  name: 'move_arm',
  description: 'Move the robot arm to a target position or pose',
  parameters: {
    type: Type.OBJECT,
    properties: {
      pose: { 
        type: Type.STRING, 
        description: 'Named pose: home, sleep, or ready',
        enum: ['home', 'sleep', 'ready']
      },
      joints: { 
        type: Type.ARRAY, 
        description: 'List of 6 joint angles (auto-detects radians or degrees)',
        items: { type: Type.NUMBER }
      },
      position: { 
        type: Type.ARRAY, 
        description: 'Cartesian position [x,y,z] in meters or [y,x] normalized',
        items: { type: Type.NUMBER }
      },
      orientation: { 
        type: Type.ARRAY, 
        description: 'Optional orientation [roll,pitch,yaw] in radians',
        items: { type: Type.NUMBER }
      },
      unit: { 
        type: Type.STRING, 
        description: 'Unit for joint angles: auto, radians, or degrees',
        enum: ['auto', 'radians', 'degrees']
      },
      moving_time: { 
        type: Type.NUMBER, 
        description: 'Time to complete movement in seconds'
      }
    },
    required: [],
  },
};

const toolGetArmStatus: FunctionDeclaration = {
  name: 'get_arm_status',
  description: 'Get current arm state, joint positions, and end effector pose',
  parameters: { type: Type.OBJECT, properties: {}, required: [] },
};

// Trajectory-based movement for complex manipulation
const toolMoveArmTrajectory: FunctionDeclaration = {
  name: 'move_arm_trajectory',
  description: 'Execute a multi-point trajectory for complex movements with optional gripper actions',
  parameters: {
    type: Type.OBJECT,
    properties: {
      trajectory: {
        type: Type.ARRAY,
        description: 'Array of waypoints forming the trajectory',
        items: {
          type: Type.OBJECT,
          properties: {
            point: {
              type: Type.ARRAY,
              description: 'Position [x,y,z] in meters or [y,x] normalized (0-1000)',
              items: { type: Type.NUMBER }
            },
            label: {
              type: Type.STRING,
              description: 'Descriptive label for this waypoint (e.g., "approach", "grasp", "lift")'
            },
            gripper_action: {
              type: Type.STRING,
              description: 'Optional gripper action at this waypoint',
              enum: ['open', 'close', 'maintain']
            }
          },
          required: ['point']
        }
      },
      speed: {
        type: Type.STRING,
        description: 'Overall trajectory execution speed',
        enum: ['slow', 'medium', 'fast']
      }
    },
    required: ['trajectory']
  }
};

// Visual object detection and targeting
const toolDetectAndTarget: FunctionDeclaration = {
  name: 'detect_and_target_object',
  description: 'Use visual input to identify objects and generate approach trajectory',
  parameters: {
    type: Type.OBJECT,
    properties: {
      object_description: {
        type: Type.STRING,
        description: 'Natural language description of target object'
      },
      action: {
        type: Type.STRING,
        description: 'Intended action with the object',
        enum: ['approach', 'grasp', 'push', 'point_to']
      },
      approach_height: {
        type: Type.NUMBER,
        description: 'Height offset above object for approach (meters, default 0.05)'
      }
    },
    required: ['object_description', 'action']
  }
};

// Scene analysis for spatial understanding
const toolAnalyzeWorkspace: FunctionDeclaration = {
  name: 'analyze_workspace',
  description: 'Analyze current workspace to identify objects and spatial relationships',
  parameters: {
    type: Type.OBJECT,
    properties: {
      analysis_type: {
        type: Type.STRING,
        description: 'Type of analysis to perform',
        enum: ['object_detection', 'spatial_map', 'obstacle_check']
      }
    },
    required: []
  }
};

// Complete autonomous pick-and-place workflow with vision
const toolPickAndPlace: FunctionDeclaration = {
  name: 'pick_and_place',
  description: 'Execute complete autonomous pick-and-place workflow: visually detect object, approach and grasp it, detect target location, move and release. Uses computer vision for object detection and grasp verification.',
  parameters: {
    type: Type.OBJECT,
    properties: {
      object_to_pick: {
        type: Type.STRING,
        description: 'Natural language description of object to pick up (e.g., "banana", "red block", "pen")'
      },
      target_location: {
        type: Type.STRING,
        description: 'Natural language description of target location to place object (e.g., "bowl", "blue container", "table")'
      },
      approach_height: {
        type: Type.NUMBER,
        description: 'Height offset above object for approach phase (meters, default 0.05)'
      },
      lift_height: {
        type: Type.NUMBER,
        description: 'Height to lift object after grasping (meters, default 0.15)'
      },
      speed: {
        type: Type.STRING,
        description: 'Overall execution speed for the workflow',
        enum: ['slow', 'medium', 'fast']
      }
    },
    required: ['object_to_pick', 'target_location']
  }
};

const SYSTEM_INSTRUCTION = `
You are an advanced spatial reasoning AI controlling a Mobile ALOHA robot with a 6-DOF arm and gripper. You have continuous visual input from a merged camera view showing both:
- LEFT GRIPPER camera - mounted on end effector for close-up manipulation
- TOP VIEW camera - overhead workspace view for spatial understanding

SPATIAL CAPABILITIES:
- Use visual input to detect, track, and analyze objects in 3D space
- Generate smooth trajectories for complex manipulation tasks
- Understand spatial relationships and plan multi-step actions
- Execute precise pick-and-place operations with visual feedback

AUTONOMOUS WORKFLOWS (HIGHEST LEVEL):
- pick_and_place(): Complete autonomous pick-and-place with vision
  * Detects object using computer vision
  * Approaches, grasps with verification
  * Detects target location visually
  * Moves and releases object
  * Verifies task completion
  * Use for: "pick the banana and put it in the bowl"
  * Example: pick_and_place(object_to_pick="banana", target_location="bowl")

TRAJECTORY-BASED CONTROL:
- move_arm_trajectory(): Execute multi-waypoint paths with labeled steps
  * Each waypoint can include gripper actions (open/close/maintain)
  * Use descriptive labels like "approach", "grasp", "lift", "place"
  * Supports speed control (slow/medium/fast)

- detect_and_target_object(): Identify objects visually and generate approach
  * Use natural language to describe target objects
  * Automatically generates appropriate trajectory

- analyze_workspace(): Understand scene layout and spatial relationships

STANDARD CONTROLS:
- move_arm(): Single-point movements (pose/joints/position)
- control_gripper(): Open or close gripper
- get_arm_status() / get_gripper_status(): Query current state

WORKSPACE CONSTRAINTS:
- x, y: [-0.5, 0.5] meters
- z: [0.1, 0.6] meters (NEVER go below z=0.1m)
- Use spatial understanding to avoid collisions

ADVANCED BEHAVIORS:
- For "pick the banana and put it in the bowl":
  1. Use pick_and_place(object_to_pick="banana", target_location="bowl")
  2. System handles detection, grasping, placement, and verification automatically

- For "pick up the red object":
  1. Use pick_and_place() for autonomous operation, OR
  2. Use detect_and_target_object() + move_arm_trajectory() for manual control

- For "organize the workspace":
  1. Analyze spatial layout with analyze_workspace()
  2. Execute sequential pick_and_place() calls for each object

- For complex custom trajectories:
  1. Break down into trajectory waypoints
  2. Label each step for clarity
  3. Coordinate arm and gripper actions with move_arm_trajectory()

TRAJECTORY FORMAT:
When using move_arm_trajectory, structure waypoints as:
[
  {"point": [x, y, z], "label": "approach", "gripper_action": "open"},
  {"point": [x, y, z], "label": "grasp_position", "gripper_action": "close"},
  {"point": [x, y, z], "label": "lift", "gripper_action": "maintain"},
  {"point": [x, y, z], "label": "place_position", "gripper_action": "open"}
]

The system automatically converts between coordinate formats and detects angle units.
`;

interface ApiMessage {
  timestamp: string;
  type: string;
  data: any;
}

export function ALOHAControl() {
  const { client, setConfig, connected } = useLiveAPIContext();
  const [taskStatus, setTaskStatus] = useState('Ready');
  const [gripperState, setGripperState] = useState<any>({
    state: 'unknown',
    position_normalized: 0,
  });
  const [armState, setArmState] = useState<any>({
    state: 'unknown',
    pose: null,
    joints_degrees: [0, 0, 0, 0, 0, 0],
  });
  const [apiMessages, setApiMessages] = useState<ApiMessage[]>([]);

  // Configure tools and system instruction before connecting
  useEffect(() => {
    setConfig({
      tools: [{ functionDeclarations: [
        // Autonomous workflows (highest level)
        toolPickAndPlace,
        // Trajectory and spatial tools (primary)
        toolMoveArmTrajectory,
        toolDetectAndTarget,
        toolAnalyzeWorkspace,
        // Standard control tools
        toolControlGripper,
        toolGetGripperStatus,
        toolMoveArm,
        toolGetArmStatus,
      ] }],
      systemInstruction: SYSTEM_INSTRUCTION,
    });
  }, [setConfig]);

  // Listen for all API messages
  useEffect(() => {
    const handleMessage = (message: any) => {
      const timestamp = new Date().toLocaleTimeString();
      setApiMessages(prev => [...prev, {
        timestamp,
        type: 'message',
        data: message
      }]);
    };

    const handleToolCall = (toolCall: any) => {
      const timestamp = new Date().toLocaleTimeString();
      setApiMessages(prev => [...prev, {
        timestamp,
        type: 'toolCall',
        data: toolCall
      }]);
    };

    const handleAudio = (audio: any) => {
      const timestamp = new Date().toLocaleTimeString();
      setApiMessages(prev => [...prev, {
        timestamp,
        type: 'audio',
        data: { received: true, size: audio.length }
      }]);
    };

    const handleError = (error: any) => {
      const timestamp = new Date().toLocaleTimeString();
      setApiMessages(prev => [...prev, {
        timestamp,
        type: 'error',
        data: error
      }]);
    };

    const handleOpen = () => {
      const timestamp = new Date().toLocaleTimeString();
      setApiMessages(prev => [...prev, {
        timestamp,
        type: 'open',
        data: { status: 'Connected to Gemini API' }
      }]);
    };

    const handleClose = () => {
      const timestamp = new Date().toLocaleTimeString();
      setApiMessages(prev => [...prev, {
        timestamp,
        type: 'close',
        data: { status: 'Disconnected from Gemini API' }
      }]);
    };

    client.on('message', handleMessage);
    client.on('toolCall', handleToolCall);
    client.on('audio', handleAudio);
    client.on('error', handleError);
    client.on('open', handleOpen);
    client.on('close', handleClose);

    return () => {
      client.off('message', handleMessage);
      client.off('toolCall', handleToolCall);
      client.off('audio', handleAudio);
      client.off('error', handleError);
      client.off('open', handleOpen);
      client.off('close', handleClose);
    };
  }, [client]);

  // Poll robot status periodically to keep UI updated
  useEffect(() => {
    const pollStatus = async () => {
      if (connected) {
        try {
          const res = await fetch(`${ROBOT_ENDPOINT}/status`);
          const data = await res.json();
          if (data.arm) {
            setArmState(data.arm);
          }
          if (data.gripper) {
            setGripperState({
              state: data.gripper.state,
              position_normalized: (data.gripper.position_percent || 0) / 100
            });
          }
        } catch (e) {
          console.error('Status poll error:', e);
        }
      }
    };

    const interval = setInterval(pollStatus, 2000); // Poll every 2 seconds
    return () => clearInterval(interval);
  }, [connected]);


  // Handle tool calls from Gemini
  useEffect(() => {
    const handleToolCall = async (toolCall: any) => {
      console.log('🤖 Tool call received:', toolCall);
      const responses: any[] = [];
      
      for (const call of toolCall.functionCalls || []) {
        setTaskStatus(`Executing: ${call.name}`);
        
        // Always respond to Gemini immediately with success
        responses.push({ 
          name: call.name, 
          id: call.id, 
          response: { success: true, status: 'executed' } 
        });
        
        // Send to Python bridge asynchronously (fire-and-forget)
        try {
          const res = await fetch(`${ROBOT_ENDPOINT}/aloha-tool-call`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: call.name, args: call.args, id: call.id }),
          });
          const data = await res.json();
          
          // Update local state with the result
          if ((call.name === 'get_gripper_status' || call.name === 'control_gripper') && data.result) {
            setGripperState(data.result);
            console.log('📊 Gripper state:', data.result);
          } else if ((call.name === 'get_arm_status' || call.name === 'move_arm') && data.result) {
            setArmState(data.result);
            console.log('🦾 Arm state:', data.result);
          } else if (call.name === 'get_robot_status' && data.result) {
            // Update both states from combined status
            if (data.result.gripper) {
              setGripperState({
                state: data.result.gripper.state,
                position_normalized: (data.result.gripper.position_percent || 0) / 100
              });
            }
            if (data.result.arm) {
              setArmState(data.result.arm);
            }
            console.log('🤖 Robot status:', data.result);
          } else if (data.result) {
            console.log(`✅ ${call.name} result:`, data.result);
          }
        } catch (e: any) {
          console.error(`❌ Bridge error for ${call.name}:`, e?.message || 'fetch error');
        }
      }
      
      // Send tool response back to Gemini
      if (responses.length > 0) {
        console.log('Sending tool response to Gemini...', responses);
        client.sendToolResponse(responses);
      }
      setTaskStatus('Ready');
    };

    client.on('toolCall', handleToolCall);
    return () => {
      client.off('toolCall', handleToolCall);
    };
  }, [client]);

  const executeTask = (text: string) => {
    if (connected) client.send({ text });
  };

  // Calculate gripper percentage for display
  const gripperPercent = (gripperState.position_normalized || 0) * 100;
  
  // Determine gripper status emoji
  const getGripperEmoji = () => {
    if (gripperState.state === 'open') return '🤚';
    if (gripperState.state === 'closed') return '✊';
    if (gripperState.state === 'opening') return '🔄';
    if (gripperState.state === 'closing') return '🔄';
    return '❓';
  };

  return (
    <div style={{ padding: 12, margin: 10, background: '#1f2937', color: 'white', borderRadius: 8 }}>
      <h3>🤖 ALOHA Robot Control</h3>
      
      {/* Status Information */}
      <div style={{ marginBottom: 12 }}>
        <div><strong>Connection:</strong> {connected ? '✅ Connected' : '❌ Disconnected'}</div>
        <div><strong>Task Status:</strong> {taskStatus}</div>
        <div><strong>Bridge Endpoint:</strong> {ROBOT_ENDPOINT}</div>
      </div>

      {/* Arm State Display */}
      <div style={{ background: '#111827', padding: 12, borderRadius: 6, marginBottom: 12 }}>
        <h4 style={{ margin: '0 0 8px 0' }}>🦾 Arm State</h4>
        <div><strong>Status:</strong> {armState.state?.toUpperCase() || 'UNKNOWN'}</div>
        <div><strong>Pose:</strong> {armState.pose || 'custom'}</div>
        <div><strong>Joints (°):</strong> {armState.joints_degrees ? 
          armState.joints_degrees.map((j: number) => j.toFixed(1)).join(', ') : 'unknown'}</div>
      </div>

      {/* Gripper State Display */}
      <div style={{ background: '#111827', padding: 12, borderRadius: 6, marginBottom: 12 }}>
        <h4 style={{ margin: '0 0 8px 0' }}>Gripper State {getGripperEmoji()}</h4>
        <div><strong>Status:</strong> {gripperState.state?.toUpperCase() || 'UNKNOWN'}</div>
        <div><strong>Position:</strong> {gripperPercent.toFixed(1)}% open</div>
        
        {/* Visual Progress Bar */}
        <div style={{ marginTop: 8 }}>
          <div style={{ 
            background: '#374151', 
            height: 20, 
            borderRadius: 10,
            overflow: 'hidden',
            position: 'relative'
          }}>
            <div style={{
              background: gripperState.state === 'closed' ? '#ef4444' : '#10b981',
              width: `${gripperPercent}%`,
              height: '100%',
              transition: 'width 0.3s ease',
            }} />
            <div style={{
              position: 'absolute',
              top: 0,
              left: 0,
              right: 0,
              textAlign: 'center',
              lineHeight: '20px',
              fontSize: 12,
              fontWeight: 'bold',
            }}>
              {gripperPercent.toFixed(0)}%
            </div>
          </div>
        </div>
      </div>


      {/* Manual Control Buttons */}
      <div style={{ marginBottom: 12 }}>
        <h4 style={{ margin: '8px 0' }}>Manual Controls</h4>
        
        {/* Arm Controls */}
        <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap', marginBottom: 8 }}>
          <button onClick={() => executeTask('Move arm to home position')}>
            🏠 Home
          </button>
          <button onClick={() => executeTask('Move arm to ready position')}>
            ✅ Ready
          </button>
          <button onClick={() => executeTask('Move arm to sleep position')}>
            😴 Sleep
          </button>
          <button onClick={() => executeTask('Move arm forward 10 centimeters')}>
            ⬆️ Forward
          </button>
          <button onClick={() => executeTask('Get arm status')}>
            📊 Arm Status
          </button>
        </div>
        
        {/* Gripper Controls */}
        <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
          <button onClick={() => executeTask('Open the gripper')}>
            🤚 Open Gripper
          </button>
          <button onClick={() => executeTask('Close the gripper')}>
            ✊ Close Gripper
          </button>
          <button onClick={() => executeTask('Pick up the object in front of you')}>
            🎯 Pick Object
          </button>
        </div>
      </div>

      {/* Voice Command Examples */}
      <div style={{ marginTop: 12, fontSize: 12, opacity: 0.8 }}>
        <div><strong>Try saying:</strong></div>
        <div>• "Move the arm to home position"</div>
        <div>• "Move forward 20 centimeters"</div>
        <div>• "Open the gripper and pick up the object"</div>
        <div>• "Move to position x=0.3, y=0, z=0.2"</div>
        <div>• "Set joint angles to 0, -55, 66, 0, -17, 0 degrees"</div>
      </div>

      {/* API Response Viewer */}
      <ApiResponseViewer
        messages={apiMessages}
        onClear={() => setApiMessages([])}
      />
    </div>
  );
}

