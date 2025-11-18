// Pen detection configuration for Live API Console

export const penDetectionTool = {
  functionDeclarations: [{
    name: "holding_pen",
    description: "Report whether a pen is visible in the video",
    parameters: {
      type: "object",
      properties: {
        is_holding: {
          type: "boolean",
          description: "True if pen is visible"
        }
      },
      required: ["is_holding"]
    }
  }]
};

export const penDetectionSystemInstruction = 
  "You see a live video stream. Every time you're asked, call holding_pen " +
  "with is_holding=true if you see a pen, is_holding=false otherwise. " +
  "Only call the function, no text responses.";

export const handlePenDetectionToolCall = (toolCall) => {
  console.log("Tool call received:", toolCall);
  
  const functionCalls = toolCall.functionCalls || [];
  for (const call of functionCalls) {
    if (call.name === "holding_pen") {
      const isHolding = call.args?.is_holding || false;
      const emoji = isHolding ? "✅" : "❌";
      const status = isHolding ? "PEN DETECTED" : "NO PEN";
      
      // Update UI or log
      console.log(`${emoji} ${status}`);
      
      // You could update a state variable here to show in the UI
      document.title = `${emoji} ${status}`;
    }
  }
};
