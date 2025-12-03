import asyncio
import aiohttp
import json
import logging
import sys

# Configure logging
logging.basicConfig(level=logging.INFO, format='[TestClient] %(message)s')

async def test_bridge():
    uri = "ws://localhost:8082/ws"
    
    async with aiohttp.ClientSession() as session:
        try:
            async with session.ws_connect(uri) as ws:
                logging.info(f"Connected to {uri}")

                # 1. Wait for connection info
                msg = await ws.receive_json()
                logging.info(f"Received: {msg}")
                
                if msg.get('type') != 'connection_info':
                    logging.error("Expected connection_info")
                    return

                # 2. Send robot_connect
                logging.info("Sending robot_connect...")
                await ws.send_json({"type": "robot_connect"})

                # 3. Wait for robot_status
                while True:
                    msg = await ws.receive_json()
                    logging.info(f"Received: {msg}")
                    if msg.get('type') == 'robot_status' and msg.get('payload', {}).get('connected'):
                        logging.info("Robot connected!")
                        break
                    if msg.get('type') == 'error':
                        logging.error(f"Error: {msg}")
                        return

                # 4. Send task_request
                task_prompt = "Pick up the red block and move it to the right."
                logging.info(f"Sending task_request: {task_prompt}")
                await ws.send_json({
                    "type": "task_request", 
                    "payload": {"prompt": task_prompt}
                })

                # 5. Listen for updates
                logging.info("Listening for updates...")
                start_time = asyncio.get_event_loop().time()
                
                while True:
                    # Timeout after 60 seconds of silence
                    try:
                        msg = await ws.receive_json(timeout=60)
                        logging.info(f"Received: {msg['type']} - {str(msg.get('payload'))[:100]}...")
                        
                        if msg.get('type') == 'task_complete':
                            logging.info("Task complete!")
                            break
                            
                        if msg.get('type') == 'error':
                            logging.error(f"Received error: {msg}")
                            break
                            
                    except asyncio.TimeoutError:
                        logging.error("Timeout waiting for message")
                        break
                        
        except aiohttp.ClientError as e:
            logging.error(f"Connection failed: {e}")
        except Exception as e:
            logging.error(f"Unexpected error: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(test_bridge())
    except KeyboardInterrupt:
        pass
