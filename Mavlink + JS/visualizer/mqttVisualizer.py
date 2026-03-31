import paho.mqtt.client as paho
import json
import ssl
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import math

class VisualizadorRuta:
    def __init__(self, max_puntos=1000):
        # Almacenar datos
        self.lats = deque(maxlen=max_puntos)
        self.lons = deque(maxlen=max_puntos)
        self.altitudes = deque(maxlen=max_puntos)
        self.tiempos = deque(maxlen=max_puntos)
        self.inicio_tiempo = time.time()
        
        # Configurar la figura
        plt.ion()  # Modo interactivo
        self.fig = plt.figure(figsize=(14, 8))
        
        # Crear subplots
        self.ax_mapa = plt.subplot(1, 2, 1)
        self.ax_altitud = plt.subplot(1, 2, 2)
        
        # Configurar mapa
        self.ax_mapa.set_xlabel('Longitud')
        self.ax_mapa.set_ylabel('Latitud')
        self.ax_mapa.set_title('Ruta de Vuelo del Dron')
        self.ax_mapa.grid(True, alpha=0.3)
        self.ax_mapa.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        self.ax_mapa.axvline(x=0, color='k', linestyle='-', alpha=0.3)
        
        # Configurar gráfico de altitud
        self.ax_altitud.set_xlabel('Tiempo (segundos)')
        self.ax_altitud.set_ylabel('Altitud (m)')
        self.ax_altitud.set_title('Perfil de Altitud')
        self.ax_altitud.grid(True, alpha=0.3)
        
        # Inicializar líneas
        self.linea_ruta, = self.ax_mapa.plot([], [], 'b-', linewidth=2, label='Ruta')
        self.punto_actual, = self.ax_mapa.plot([], [], 'ro', markersize=8, label='Posición actual')
        self.inicio, = self.ax_mapa.plot([], [], 'g*', markersize=12, label='Inicio')
        
        self.linea_altitud, = self.ax_altitud.plot([], [], 'g-', linewidth=2, label='Altitud')
        self.punto_altitud, = self.ax_altitud.plot([], [], 'ro', markersize=6)
        
        # Mostrar leyendas
        self.ax_mapa.legend(loc='upper right')
        self.ax_altitud.legend(loc='upper right')
        
        # Variables de estado
        self.ultima_actualizacion = 0
        self.contador_puntos = 0
        self.distancia_total = 0
        self.ultimo_punto = None
        
        plt.tight_layout()
        plt.show()
        
    def calcular_distancia(self, lat1, lon1, lat2, lon2):
        """Calcula distancia entre dos puntos en metros"""
        R = 6371000  # Radio de la Tierra en metros
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi/2)**2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
        
    def actualizar(self, lat, lon, alt, tiempo):
        """Actualiza el gráfico con nuevos datos"""
        if lat is None or lon is None or lat == 0 or lon == 0:
            return
            
        # Agregar datos
        self.lats.append(lat)
        self.lons.append(lon)
        self.altitudes.append(alt if alt else 0)
        
        tiempo_relativo = tiempo - self.inicio_tiempo if tiempo else time.time() - self.inicio_tiempo
        self.tiempos.append(tiempo_relativo)
        
        # Calcular distancia recorrida
        if self.ultimo_punto is not None:
            self.distancia_total += self.calcular_distancia(
                self.ultimo_punto[0], self.ultimo_punto[1],
                lat, lon
            )
        
        self.ultimo_punto = (lat, lon)
        self.contador_puntos += 1
        
        # Actualizar gráfico del mapa
        self.linea_ruta.set_data(list(self.lons), list(self.lats))
        self.punto_actual.set_data([lon], [lat])
        
        # Actualizar punto de inicio
        if len(self.lats) > 0:
            self.inicio.set_data([self.lons[0]], [self.lats[0]])
        
        # Ajustar límites del mapa con margen
        if len(self.lats) > 1:
            margen_lat = (max(self.lats) - min(self.lats)) * 0.1
            margen_lon = (max(self.lons) - min(self.lons)) * 0.1
            
            self.ax_mapa.set_xlim(min(self.lons) - margen_lon, max(self.lons) + margen_lon)
            self.ax_mapa.set_ylim(min(self.lats) - margen_lat, max(self.lats) + margen_lat)
        else:
            self.ax_mapa.set_xlim(lon - 0.001, lon + 0.001)
            self.ax_mapa.set_ylim(lat - 0.001, lat + 0.001)
        
        # Actualizar gráfico de altitud
        self.linea_altitud.set_data(list(self.tiempos), list(self.altitudes))
        self.punto_altitud.set_data([tiempo_relativo], [alt])
        
        # Ajustar límites de altitud
        if len(self.altitudes) > 1:
            margen_alt = (max(self.altitudes) - min(self.altitudes)) * 0.1
            self.ax_altitud.set_ylim(min(self.altitudes) - margen_alt, max(self.altitudes) + margen_alt)
        else:
            self.ax_altitud.set_ylim(alt - 10, alt + 10)
        
        self.ax_altitud.set_xlim(0, max(self.tiempos) + 2)
        
        # Actualizar título con estadísticas
        self.ax_mapa.set_title(f'Ruta de Vuelo | Distancia: {self.distancia_total/1000:.2f} km | Puntos: {self.contador_puntos}')
        
        # Refrescar gráfico
        plt.draw()
        plt.pause(0.01)
        
        # Mostrar información en consola cada 10 puntos
        if self.contador_puntos % 10 == 0:
            print(f"📍 Posición: ({lat:.7f}, {lon:.7f}) | Alt: {alt:.1f}m | Distancia: {self.distancia_total/1000:.2f}km | Puntos: {self.contador_puntos}")

# Variable global para el visualizador
visualizador = None

def on_message(client, userdata, msg):
    global visualizador
    try:
        data = json.loads(msg.payload.decode())
        
        lat = data.get('latitude')
        lon = data.get('longitude')
        alt = data.get('altitude_asl')
        tiempo = data.get('time')
        
        # Convertir tiempo si es string
        if tiempo and isinstance(tiempo, str):
            try:
                tiempo = float(tiempo)
            except:
                tiempo = time.time()
        
        if visualizador and lat and lon:
            visualizador.actualizar(lat, lon, alt, tiempo)
                
    except Exception as e:
        print(f"Error: {e}")

# Configurar MQTT
client = paho.Client(
    client_id='VisualizadorRuta',
    transport='websockets',
    callback_api_version=paho.CallbackAPIVersion.VERSION1
)
client.username_pw_set('bcn', 'Barcelona_1234')
context = ssl.create_default_context()
client.tls_set_context(context)
client.ws_set_options(path="/mqtt/")
client.on_message = on_message

print("="*60)
print("🚁 VISUALIZADOR DE RUTA EN TIEMPO REAL")
print("="*60)
print("\n🔌 Conectando al broker MQTT...")

try:
    client.connect('mqtt.catuav.com', 443)
    client.subscribe('drone_aerowatch_telemetry')
    print("✅ Conectado exitosamente!")
    print("\n📊 Se abrirá una ventana con el gráfico de la ruta")
    print("   - Gráfico izquierdo: Ruta en 2D (latitud vs longitud)")
    print("   - Gráfico derecho: Perfil de altitud")
    print("💡 La ventana se actualizará automáticamente")
    print("💡 Presiona Ctrl+C en la terminal para salir\n")
    
    # Inicializar visualizador
    visualizador = VisualizadorRuta()
    
    # Iniciar loop
    client.loop_forever()
    
except Exception as e:
    print(f"❌ Error de conexión: {e}")
except KeyboardInterrupt:
    print("\n\n👋 Programa finalizado")
    if visualizador:
        plt.close('all')