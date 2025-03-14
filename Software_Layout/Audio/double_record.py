# FOR SENDING AUDIO FILES FROM MIC TO CATHERINE, HERS IS AUDIORECEIVER.PY

import os
import time
import asyncio
import websockets
import threading

websocket_url = "ws://192.168.7.86:8764"
websocket = None

async def send_audio(file_name, websocket_url):
    async with websockets.connect(websocket_url) as websocket:
        with open(file_name, "rb") as f:
            data = f.read()
            await websocket.send(data)
            print(f"{file_name} sent to {websocket_url}")

def record_audio(file_name):
    command = f"arecord -r 48000 -f S16_LE -d 5 {file_name}"
    os.system(command)

def run_event_loop():
    """Run the asyncio event loop in a separate thread"""
    asyncio.set_event_loop(loop)
    loop.run_forever()

async def connect_websocket(self):
    """Ensure the WebSocket connection is active"""
    try:
        if websocket is None or websocket.close_code is not None:
            self.get_logger().info("Connecting to WebSocket server...")
            websocket = await websockets.connect(websocket_url)
    except Exception as e:
        self.get_logger().error(f"WebSocket connection failed: {str(e)}")
        websocket = None  # Reset to force retry



async def send_packet(self):
    """Send mock data to the WebSocket server"""
    await connect_websocket()  # Ensure WebSocket is connected

    if websocket:
        try:
            await send_audio()
            self.get_logger().info(f"Sent audio file")
        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().error(f"WebSocket closed unexpectedly: {str(e)}")
            websocket = None  # Force reconnect next attempt
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {str(e)}")
            websocket = None  # Reset connection

async def main():
    for i in range(3):  # You can increase the range if you want more recordings
        file_name = f"audio_{i+1}.wav"
        print(f"Recording: {file_name}")
        record_audio(file_name)
        print(f"Recording {file_name} completed.")
        await send_audio(file_name, websocket_url)
        #await asyncio.sleep(0.001)  # Adding a 1-millisecond delay

if __name__ == "__main__":
        # Start the asyncio event loop in a separate thread
    loop = asyncio.new_event_loop()
    loop_thread = threading.Thread(target=run_event_loop, daemon=True)
    loop_thread.start()
    asyncio.run(main())