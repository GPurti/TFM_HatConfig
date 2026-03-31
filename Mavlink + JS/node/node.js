const mqtt = require("mqtt");

const brokerUrl = "wss://mqtt.catuav.com:443/mqtt/"; 
const mqttUser = "bcn";
const mqttPass = "Barcelona_1234";
const topicSub = "drone_aerowatch_action";

const pythonUrl = "http://127.0.0.1:5000/action";

const client = mqtt.connect(brokerUrl, {
  username: mqttUser,
  password: mqttPass,
  protocol: "wss",
  reconnectPeriod: 2000,
});

client.on("connect", () => {
  console.log("✅ Conectado a broker MQTT");
  client.subscribe(topicSub, (err) => {
    if (err) {
      console.error("❌ Error al suscribirse:", err);
    } else {
      console.log(`📡 Suscrito al topic: ${topicSub}`);
    }
  });
});

client.on("message", async (topic, message) => {
  const msg = message.toString();
  console.log(`📩 Recibido MQTT en ${topic}:`, msg);

  try {
    const response = await fetch(pythonUrl, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ topic, msg }),
    });

    console.log("➡️ Enviado a Python, status:", response.status);
  } catch (err) {
    console.error("❌ Error enviando a Python:", err);
  }
});

client.on("error", (err) => {
  console.error("Error MQTT:", err);
});
