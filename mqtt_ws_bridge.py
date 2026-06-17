import json
import asyncio
import paho.mqtt.client as mqtt


class MQTTWebSocketBridge:
    def __init__(self, broker="localhost", port=1883, topics=None):
        self.broker = broker
        self.port = port
        self.topics = topics or ["/api/#"]

        self.clients = set()
        self.loop = None

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

    # -------- MQTT --------
    def on_connect(self, client, userdata, flags, rc):
        print("MQTT Connected")
        for topic in self.topics:
            client.subscribe(topic)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
        except:
            payload = msg.payload.decode()

        data = {
            "topic": msg.topic,
            "payload": payload
        }

        # Push to all WS clients (thread-safe)
        if self.loop:
            asyncio.run_coroutine_threadsafe(
                self.broadcast(data),
                self.loop
            )

    # -------- WebSocket --------
    async def register(self, websocket):
        self.clients.add(websocket)

    async def unregister(self, websocket):
        self.clients.discard(websocket)

    async def broadcast(self, data):
        dead = []

        for ws in self.clients:
            try:
                await ws.send_text(json.dumps(data))
            except:
                dead.append(ws)

        # cleanup dead connections
        for ws in dead:
            self.clients.discard(ws)

    # -------- START --------
    def start(self, loop):
        self.loop = loop
        self.mqtt_client.connect(self.broker, self.port, 60)
        self.mqtt_client.loop_start()

