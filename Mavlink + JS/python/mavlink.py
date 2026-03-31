#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pymavlink import mavutil
import time
from datetime import datetime
import json
from os import system
import ssl
import paho.mqtt.client as paho
import math
import threading
from flask import Flask, request

app = Flask(__name__)

# ============================================================
# NUEVO: Variables para comandos de emergencia (SIN COLAS)
# ============================================================
emergencia_en_curso = False
emergencia_lock = threading.Lock()

# ============================================================

def send_waypoint_custom(seq, wp, frame, cmd, is_first=False):
    """Envía un comando de misión con tipo específico (ej. TAKEOFF, RTL, etc.)"""
    mavlog.mav.mission_item_int_send(
        mavlog.target_system,
        mavlog.target_component,
        seq,
        frame,
        cmd,
        1 if is_first else 0,  # current
        1,  # autocontinue
        0,  # param1: hold time / min pitch / delay
        0,  # param2: acceptance radius / climb rate
        0,  # param3: pass radius / unused
        0,  # param4: yaw / heading
        int(wp['lat'] * 1e7) if 'lat' in wp else 0,
        int(wp['lon'] * 1e7) if 'lon' in wp else 0,
        float(wp['alt']) if 'alt' in wp else 0,
        0  # mission_type
    )
    print(f"Waypoint {seq} - CMD: {cmd} - Lat: {wp.get('lat')}, Lon: {wp.get('lon')}, Alt: {wp.get('alt')}")

def connection():
    global mavlog
    # SOPORTE PARA MÚLTIPLES MODOS - DETECCIÓN AUTOMÁTICA
    import sys
    import platform
    
    sistema = platform.system()
    
    # Si se pasa como argumento, usar ese modo
    if len(sys.argv) > 1:
        modo = sys.argv[1]
        if modo == 'tcp':
            mavlog = mavutil.mavlink_connection('tcp:127.0.0.1:5762', wait_ready=True, baud=115200)
        elif modo == 'udp':
            mavlog = mavutil.mavlink_connection('udp:127.0.0.1:14550', wait_ready=True, baud=115200)
        elif modo == 'serial':
            mavlog = mavutil.mavlink_connection('/dev/serial0', wait_ready=True, baud=57600)
        else:
            mavlog = mavutil.mavlink_connection(modo, wait_ready=True, baud=115200)
    else:
        # Auto-detección
        if sistema == "Windows":
            print("Windows detectado - Usando TCP (modo prueba)")
            mavlog = mavutil.mavlink_connection('tcp:127.0.0.1:5762', wait_ready=True, baud=115200)
        else:
            print("Linux detectado - Usando serie /dev/serial0")
            mavlog = mavutil.mavlink_connection('/dev/serial0', wait_ready=True, baud=57600)
    
    time.sleep(5)
    print('Connected')

def radians_to_degrees(radians):
    """Convierte radianes a grados de forma robusta"""
    try:
        return round(math.degrees(radians), 2)
    except (TypeError, ValueError):
        return 0.0

def process_mavlink_message(msg):
    global latitude, longitude, altitude_agl, altitude_ahl, altitude_asl, groundspeed, airspeed, heading, pitch, yaw, roll, gps_timestamp, system_time, attitude_time, global_position_time, voltage_battery, current_battery, battery_remaining
    
    if msg.get_type() == 'SYSTEM_TIME':
        try:
            system_time = round(msg.time_unix_usec / 1e6, 3)
        except:
            pass
    
    elif msg.get_type() == 'GPS_RAW_INT':
        try:
            gps_timestamp = round(msg.time_usec / 1e6, 3)
            latitude = round(msg.lat / 1.0e7, 7)
            longitude = round(msg.lon / 1.0e7, 7)
        except:
            pass

    elif msg.get_type() == 'VFR_HUD':
        try:
            airspeed = round(msg.airspeed, 2)
            groundspeed = round(msg.groundspeed, 2)
        except:
            pass

    elif msg.get_type() == 'ATTITUDE':
        try:
            attitude_time = round(msg.time_boot_ms / 1000.0, 3)
            pitch_deg = radians_to_degrees(msg.pitch)
            roll_deg = radians_to_degrees(msg.roll)
            yaw_deg = radians_to_degrees(msg.yaw)
            yaw = round(yaw_deg, 2)
            pitch = round(pitch_deg, 2)
            roll = round(roll_deg, 2)
        except:
            pass
    
    elif msg.get_type() == 'GLOBAL_POSITION_INT':
        try:
            global_position_time = round(msg.time_boot_ms / 1000.0, 3)
            heading = round(msg.hdg/100, 2)
            altitude_asl = round(msg.alt/1000, 2)
            altitude_ahl = round(msg.relative_alt/1000, 2)
        except:
            pass

    elif msg.get_type() == 'TERRAIN_REPORT':
        try:
            altitude_agl = round(msg.current_height, 2)
        except:
            pass
    
    elif msg.get_type() == 'BATTERY_STATUS':
        try:
            voltage_battery = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else None  
            current_battery = msg.current_battery / 100.0 if msg.current_battery != -1 else None
            battery_remaining = msg.battery_remaining if msg.battery_remaining != -1 else None
        except:
            pass

def send_mavlink_commands():
    # Solicita/activa envíos de distintos mensajes al autopiloto
    mavlog.mav.command_long_send(1, 0, 512, 0, 2, 1e6/50, 0, 0, 0, 0, 0) # SYSTEM_TIME
    mavlog.mav.command_long_send(1, 0, 512, 0, 27, 1e6/50, 0, 0, 0, 0, 0) # RAW_IMU
    mavlog.mav.command_long_send(1, 0, 512, 0, 74, 1e6/50, 0, 0, 0, 0, 0) # VFR_HUD
    mavlog.mav.command_long_send(1, 0, 512, 0, 24, 1e6/50, 0, 0, 0, 0, 0) # GPS_RAW_INT
    mavlog.mav.command_long_send(1, 0, 512, 0, 36, 1e6/50, 0, 0, 0, 0, 0) # SERVO_OUTPUT_RAW
    mavlog.mav.command_long_send(1, 0, 512, 0, 30, 1e6/50, 0, 0, 0, 0, 0) # ATTITUDE
    mavlog.mav.command_long_send(1, 0, 512, 0, 33, 1e6/50, 0, 0, 0, 0, 0) # GLOBAL_POSITION_INT
    mavlog.mav.command_long_send(1, 0, 512, 0, 136, 1e6/50, 0, 0, 0, 0, 0) # TERRAIN_REPORT
    mavlog.mav.command_long_send(1, 0, 512, 0, 143, 1e6/50, 0, 0, 0, 0, 0) # SCALED_PRESSURE3
    mavlog.mav.command_long_send(1, 0, 512, 0, 1, 1e6/50, 0, 0, 0, 0, 0) # SYS_STATUS
    mavlog.mav.command_long_send(1, 0, 512, 0, 147, 1e6/50, 0, 0, 0, 0, 0) # BATTERY_STATUS

def get_home_position():
    """Obtiene la posición home del vehículo"""
    print("Obteniendo posición home...")
    mavlog.mav.command_long_send(
        mavlog.target_system,
        mavlog.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    
    home = mavlog.recv_match(type='HOME_POSITION', blocking=True, timeout=5)
    if home is None:
        print("Error: No se recibió posición home")
        return None
    
    lat = home.latitude / 1e7
    lon = home.longitude / 1e7
    alt = home.altitude / 1000.0  # Convertir a metros
    
    print(f"Posición home obtenida: Lat={lat}, Lon={lon}, Alt={alt}")
    return {"lat": lat, "lon": lon, "alt": alt}

def send_waypoint(seq, wp, frame, is_first=False):
    """Envía un waypoint con el frame especificado"""
    mavlog.mav.mission_item_int_send(
        mavlog.target_system,
        mavlog.target_component,
        seq,
        frame,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        1 if is_first else 0,
        1,
        0, 0, 0, 0,
        int(float(wp['lat']) * 1e7),
        int(float(wp['lon']) * 1e7),
        int(float(wp['alt'])),
        0
    )
    status = "(FIRST)" if is_first else ""
    print(f"Waypoint {seq} {status}: Lat={wp['lat']}, Lon={wp['lon']}, Alt={wp['alt']}, Frame={frame}")

def mqtt_publisher(broker_pub, port_pub, topic_pub, broker_sub, port_sub, topic_sub):
    """
    Publica telemetría periódicamente al broker.
    """
    global latitude, longitude, altitude_agl, altitude_ahl, altitude_asl, groundspeed, airspeed, heading, pitch, yaw, roll, timestamp, gps_timestamp, system_time, attitude_time, global_position_time, voltage_battery, current_battery, battery_remaining
    
    # Inicializar variables
    latitude = longitude = altitude_agl = altitude_ahl = altitude_asl = None
    groundspeed = airspeed = heading = pitch = yaw = roll = None
    gps_timestamp = system_time = attitude_time = global_position_time = None
    voltage_battery = current_battery = battery_remaining = None
    
    # Crear cliente MQTT
    client = paho.Client(client_id='SIR_Client', transport='websockets', callback_api_version=paho.CallbackAPIVersion.VERSION1)
    client.username_pw_set('bcn', 'Barcelona_1234')
    context = ssl.SSLContext(protocol=ssl.PROTOCOL_TLSv1_2)
    client.tls_set_context(context)
    client.ws_set_options(path="/mqtt/")

    # Conectar al broker
    print(f"Conectando al broker MQTT: {broker_pub}")
    try:
        client.connect(broker_pub, port_pub)
        client.loop_start()
        print(f"✅ Cliente MQTT de telemetría activo. Publicando en: {topic_pub}")
    except Exception as e:
        print(f"❌ Error conectando al broker MQTT: {e}")
        return
    
    while True:
        try:
            send_mavlink_commands()
            
            # Procesar mensajes MAVLINK que hayan llegado
            while True:
                msg = mavlog.recv_match(type=['SYSTEM_TIME','RAW_IMU','VFR_HUD', 'GPS_RAW_INT', 
                                            'SERVO_OUTPUT_RAW', 'ATTITUDE', 'GLOBAL_POSITION_INT', 
                                            'TERRAIN_REPORT', 'SCALED_PRESSURE3', 'SYS_STATUS'], 
                                      blocking=False)
                if msg is None:
                    break
                process_mavlink_message(msg)
            
            if system_time is not None:
                timestamp = system_time
            else:
                timestamp = time.time()
            
            msg = {
                'time': str(timestamp),
                'gps_timestamp': gps_timestamp,
                'system_time': system_time,
                'attitude_time': attitude_time,
                'global_position_time': global_position_time,
                'latitude': latitude,
                'longitude': longitude,
                'altitude_ahl': altitude_ahl,
                'altitude_agl': altitude_agl,
                'altitude_asl': altitude_asl,
                'airSpeed': airspeed,
                'groundSpeed': groundspeed,
                'heading': heading,
                'pitch': pitch,
                'roll': roll,
                'yaw': yaw,
                'voltage_battery': voltage_battery,
                'drone_id': f"{mavlog.target_system}_{mavlog.target_component}"
            }
            payload = json.dumps(msg, indent=4)
            client.publish(topic_pub, str(payload), qos=0)
            
            time.sleep(0.1)
            
        except Exception as e:
            print(f"Error en el bucle principal: {e}")
            import traceback
            traceback.print_exc()

# ============================================================
# NUEVO: Función para comandos de emergencia (CON ARM Y TAKEOFF)
# ============================================================

def ejecutar_emergencia(action):
    """Ejecuta un comando de emergencia (interrumpe misión si es necesario)"""
    global emergencia_en_curso
    
    with emergencia_lock:
        if emergencia_en_curso:
            print("⚠️ Ya hay una emergencia en curso...")
            return
        emergencia_en_curso = True
    
    try:
        print(f"\n🚨 EJECUTANDO COMANDO: {action}")
        
        # Cambiar a modo GUIDED para detener cualquier misión (excepto DISARM)
        if action not in ['DISARM']:
            mavlog.mav.command_long_send(
                mavlog.target_system, mavlog.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 0, 0, 0, 0, 0
            )
            time.sleep(1)
        
        if action == 'ARM':
            print("🔓 ARMANDO dron...")
            mavlog.mav.command_long_send(
                mavlog.target_system, mavlog.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0  # 1 = arm
            )
            print("✅ Comando ARM enviado")
            
        elif action == 'TAKEOFF':
            print("🛫 DESPEGANDO a 20m...")
            mavlog.mav.command_long_send(
                mavlog.target_system, mavlog.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, 20  # 20 metros
            )
            print("✅ Comando TAKEOFF enviado")
            
        elif action == 'DISARM':
            print("🔓 Desarmando dron...")
            mavlog.mav.command_long_send(
                mavlog.target_system, mavlog.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0
            )
            print("✅ Comando DISARM enviado")
            
        elif action == 'EMERGENCY_STOP':
            print("🛑 PARADA DE EMERGENCIA!")
            mavlog.mav.command_long_send(
                mavlog.target_system, mavlog.target_component,
                mavutil.mavlink.MAV_CMD_EMERGENCY_LAND, 0,
                0, 0, 0, 0, 0, 0, 0
            )
            
        elif action == 'RTL':
            print("🏠 Return To Launch...")
            mavlog.mav.command_long_send(
                mavlog.target_system, mavlog.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                0, 0, 0, 0, 0, 0, 0
            )
            
        elif action == 'LAND_NOW':
            print("🛬 Aterrizando inmediatamente...")
            mavlog.mav.command_long_send(
                mavlog.target_system, mavlog.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
                0, 0, 0, 0, 0, 0, 0
            )
            
        elif action == 'HOLD_POSITION':
            print("⏸️ Manteniendo posición (LOITER)...")
            mavlog.mav.command_long_send(
                mavlog.target_system, mavlog.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                5, 0, 0, 0, 0, 0  # 5 = LOITER
            )
        
        print(f"✅ Comando {action} ejecutado correctamente")
        
    except Exception as e:
        print(f"❌ Error en comando {action}: {e}")
    
    finally:
        with emergencia_lock:
            emergencia_en_curso = False

# ============================================================
# FIN DE LAS MODIFICACIONES
# ============================================================

# -------------------------
# Lógica de procesamiento de comandos (SIN MODIFICAR)
# -------------------------
def process_command(json_data):
    """
    Ejecuta la lógica que antes estaba dentro de on_message.
    json_data debe ser un dict ya parseado.
    """
    try:
        print(f"\nComando recibido para procesar: {json_data}")

        action = json_data.get('action')
        new_lat = json_data.get('new_lat')
        new_lon = json_data.get('new_lon')
        new_alt = json_data.get('new_alt')
        waypoints = json_data.get('waypoints', [])
        
        # Convertir coordenadas a entero (1e7) si vienen en grados
        if new_lat is not None and new_lon is not None:
            try:
                new_lat = int(float(new_lat) * 1e7)
                new_lon = int(float(new_lon) * 1e7)
            except Exception:
                # si ya vienen en entero o formato distinto, lo dejamos como está
                pass

        print(f"Procesando comando - Action: {action}, Lat: {new_lat}, Lon: {new_lon}, Alt: {new_alt}, Waypoints: {waypoints}")

        if action == 'GUIDED' and new_lat is not None and new_lon is not None and new_alt is not None:
            # 1. Verificar/solicitar capacidades/autopilot
            print("Verificando estado de armado / capacidades...")
            mavlog.mav.command_long_send(
                mavlog.target_system,
                mavlog.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            time.sleep(1)

            # 2. Cambiar a modo GUIDED
            print("Cambiando a modo GUIDED...")
            mavlog.mav.command_long_send(
                mavlog.target_system,
                mavlog.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4,  # GUIDED para ArduPlane (si tu autopilot usa otra numeración, ajústalo)
                0, 0, 0, 0, 0
            )
            
            ack = mavlog.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack is None or getattr(ack, 'result', None) != 0:
                print(f"Error al cambiar a modo GUIDED: {ack}")
                return
            print("Modo GUIDED establecido correctamente")
            time.sleep(2)

            # 3. Enviar comando de reposicionamiento
            print(f"Enviando comando DO_REPOSITION a Lat: {new_lat}, Lon: {new_lon}, Alt: {new_alt}")
            mavlog.mav.command_int_send(
                mavlog.target_system,
                mavlog.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0,
                0,  # Confirmation
                0,   # Param1 (speed)
                1,   # Param2 (bitmask: 1 = change to GUIDED)
                0,   # Param3
                0,   # Param4 (yaw)
                new_lat,
                new_lon,
                new_alt
            )
            
            ack = mavlog.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack is None or getattr(ack, 'result', None) != 0:
                print(f"Error al enviar comando de reposicionamiento: {ack}")
                return
            
            print("Comando de reposicionamiento enviado correctamente")

        elif action == 'AUTO':
            # Cambiar a GUIDED para modificar misión
            print("Cambiando a modo GUIDED (preparando misión AUTO)...")
            mavlog.mav.command_long_send(
                mavlog.target_system,
                mavlog.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4,
                0, 0, 0, 0, 0
            )
            
            ack = mavlog.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack is None or getattr(ack, 'result', None) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Error al cambiar a modo GUIDED: {ack}")
                return
            print("Modo GUIDED establecido correctamente")
            time.sleep(2)
            
            # Limpiar misión
            print("Limpiando misión actual...")
            mavlog.mav.mission_clear_all_send(mavlog.target_system, mavlog.target_component)
            ack = mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
            if ack is None or getattr(ack, 'type', None) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f"Error al limpiar la misión: {ack}")
                return
            print("Misión actual limpiada correctamente")
            
            # Obtener home
            home = get_home_position()
            if home is None:
                return
            
            # Preparar waypoints: HOME -> TAKEOFF -> user waypoints (+ RTL si quieres)
            takeoff_alt = waypoints[0]['alt'] if waypoints else 50
            takeoff_cmd = {'lat': home['lat'], 'lon': home['lon'], 'alt': takeoff_alt}
            all_waypoints = [home, takeoff_cmd] + waypoints

            # Enviar misión
            print(f"Enviando misión con {len(all_waypoints)} comandos")
            mavlog.mav.mission_count_send(mavlog.target_system, mavlog.target_component, len(all_waypoints))

            for i, wp in enumerate(all_waypoints):
                req = mavlog.recv_match(type=['MISSION_REQUEST','MISSION_REQUEST_INT'], blocking=True, timeout=5)
                if not req or req.seq != i:
                    print(f"Error en secuencia: esperado {i}, recibido {getattr(req,'seq',None)}")
                    return

                frame = mavutil.mavlink.MAV_FRAME_GLOBAL if i == 0 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                if i == 0:
                    cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                elif i == 1:
                    cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                elif i == len(all_waypoints) - 1:
                    cmd = mavutil.mavlink.MAV_CMD_NAV_LAND
                else:
                    cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

                send_waypoint_custom(i, wp, frame, cmd, is_first=(i == 0))
            
            ack = mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if not ack or getattr(ack, 'type', None) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f"Error en confirmación final: {ack}")
                return
            print("Misión configurada correctamente")

            # Armar
            print("Armando el dron...")
            mavlog.mav.command_long_send(mavlog.target_system, mavlog.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                         0, 1, 0, 0, 0, 0, 0, 0)
            time.sleep(2)

            # Cambiar a AUTO y arrancar misión
            print("Cambiando a modo AUTO...")
            mavlog.mav.command_long_send(mavlog.target_system, mavlog.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                         0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3, 0, 0, 0, 0, 0)
            ack = mavlog.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack is None or getattr(ack, 'result', None) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Error al cambiar a modo AUTO: {ack}")
                return
            print("Modo AUTO establecido correctamente")

            print("Iniciando la misión (NAV_START)...")
            mavlog.mav.command_long_send(mavlog.target_system, mavlog.target_component,
                                         mavutil.mavlink.MAV_CMD_MISSION_START,
                                         0, 0, 0, 0, 0, 0, 0, 0)
            ack = mavlog.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack is None or getattr(ack, 'result', None) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Error al iniciar la misión: {ack}")
                return
            print("Misión iniciada correctamente")

    except Exception as e:
        print(f"Error procesando comando: {e}")
        import traceback
        traceback.print_exc()

# -------------------------
# Flask endpoint: recibe comandos desde Node.js (HTTP POST)
# -------------------------
@app.route("/action", methods=["POST"])
def action_handler():
    try:
        json_data = request.get_json(force=True)
        if not json_data:
            return "No JSON recibido", 400

        # Si llega el campo "msg", lo parseamos
        if "msg" in json_data:
            try:
                msg = json.loads(json_data["msg"])
            except Exception as e:
                print(f"❌ Error parseando msg: {e}")
                return "Bad msg JSON", 400
        else:
            msg = json_data

        # Detectar comandos de emergencia y ejecutarlos directamente
        action = msg.get('action')
        acciones_emergencia = ['ARM', 'TAKEOFF', 'DISARM', 'EMERGENCY_STOP', 'RTL', 'LAND_NOW', 'HOLD_POSITION']
        
        if action in acciones_emergencia:
            # Ejecutar emergencia directamente (sin cola, prioridad máxima)
            t = threading.Thread(target=ejecutar_emergencia, args=(action,))
            t.daemon = True
            t.start()
            return "EMERGENCY EXECUTED", 200
        else:
            # Comandos normales (AUTO, GUIDED) van a process_command
            t = threading.Thread(target=process_command, args=(msg,))
            t.daemon = True
            t.start()
            return "OK", 200
            
    except Exception as e:
        print(f"Error en endpoint /action: {e}")
        import traceback
        traceback.print_exc()
        return "Error interno", 500

@app.route("/emergency", methods=["POST"])
def emergency_handler():
    """Endpoint específico para emergencias"""
    try:
        json_data = request.get_json(force=True) or {}
        action = json_data.get('action', 'EMERGENCY_STOP')
        
        print(f"🚨🚨🚨 EMERGENCIA ACTIVADA: {action} 🚨🚨🚨")
        
        t = threading.Thread(target=ejecutar_emergencia, args=(action,))
        t.daemon = True
        t.start()
        
        return "EMERGENCY ACTIVATED", 200
    except Exception as e:
        print(f"Error en emergencia: {e}")
        return "Error", 500

def publish_fire_detection(client, topic="fire_detection"):
    """Publica detección de incendio con coordenadas reales del dron"""
    global latitude, longitude, altitude_asl

    if latitude is None or longitude is None:
        print("⚠️ No hay GPS válido aún, no se publica fire_detection")
        return

    payload = {
        'event': 'fire_detection',
        'coordinates': {
            'lat': round(float(latitude), 8),
            'lon': round(float(longitude), 8),
            'alt': round(float(altitude_asl) if altitude_asl else 0.0, 2)
        },
        'timestamp': time.time(),
        'drone_id': f"{mavlog.target_system}_{mavlog.target_component}"
    }

    try:
        client.publish(topic, json.dumps(payload))
        print(f"[MQTT] 🔥 Fire detection publicado: {payload}")
    except Exception as e:
        print(f"[MQTT] Error publicando fire_detection: {e}")

if __name__ == "__main__":
    import sys
    
    broker = 'mqtt.catuav.com'
    port = 443
    topic_pub = 'drone_aerowatch_telemetry'
    topic_sub = 'drone_aerowatch_action'

    print("Iniciando conexión con el dron...")
    print("Modos de uso:")
    print("  python mavlink.py           -> Auto-detección (Windows=TCP, Linux=Serial)")
    print("  python mavlink.py tcp       -> Fuerza conexión TCP (127.0.0.1:5762)")
    print("  python mavlink.py udp       -> Fuerza conexión UDP (127.0.0.1:14550)")
    print("  python mavlink.py serial    -> Fuerza conexión serie (/dev/serial0)")
    print("  python mavlink.py /dev/ttyUSB0 -> Conexión a puerto específico")
    print()
    
    connection()

    print("Iniciando cliente MQTT (telemetría) en hilo separado...")
    mqtt_thread = threading.Thread(target=mqtt_publisher, args=(broker, port, topic_pub, broker, port, topic_sub))
    mqtt_thread.daemon = True
    mqtt_thread.start()

    print("Levantar servicio HTTP para recibir comandos desde Node.js en /action (puerto 5000)")
    print("📡 Endpoints disponibles:")
    print("   POST /action - Comandos normales (AUTO, GUIDED) y emergencias (ARM, TAKEOFF, DISARM, RTL, etc.)")
    print("   POST /emergency - Comandos de emergencia (máxima prioridad)")
    print("\nComandos soportados:")
    print("   AUTO - Misión con waypoints")
    print("   GUIDED - Movimiento a punto específico (requiere new_lat, new_lon, new_alt)")
    print("   ARM - Armar dron")
    print("   TAKEOFF - Despegar a 20m")
    print("   DISARM - Desarmar dron")
    print("   RTL - Return To Launch")
    print("   LAND_NOW - Aterrizar inmediatamente")
    print("   HOLD_POSITION - Mantener posición (modo LOITER)")
    print("   EMERGENCY_STOP - Parada de emergencia")
    
    app.run(host="0.0.0.0", port=5000, threaded=True)