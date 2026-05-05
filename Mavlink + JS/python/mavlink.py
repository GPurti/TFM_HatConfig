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
# Variables para comandos de emergencia
# ============================================================
emergencia_en_curso = False
emergencia_lock = threading.Lock()

# Protege el acceso concurrente al objeto mavlog entre el hilo de telemetría
# y los hilos de comandos (fence, misión...). RLock permite reentrada desde
# funciones que se llaman entre sí (p.ej. send_fence -> get_existing_fences).
mavlink_io_lock = threading.RLock()

# ============================================================
# Variables globales de telemetría
# ============================================================
latitude = longitude = altitude_agl = altitude_ahl = altitude_asl = None
groundspeed = airspeed = heading = pitch = yaw = roll = None
gps_timestamp = system_time = attitude_time = global_position_time = None
voltage_battery = current_battery = battery_remaining = None
mavlog = None
flight_mode = None
armed = None

# ============================================================
# Funciones de utilidad
# ============================================================

def radians_to_degrees(radians):
    """Convierte radianes a grados de forma robusta"""
    try:
        return round(math.degrees(radians), 2)
    except (TypeError, ValueError):
        return 0.0

def send_waypoint_custom(seq, wp, frame, cmd, is_first=False):
    """Envía un comando de misión con tipo específico"""
    mavlog.mav.mission_item_int_send(
        mavlog.target_system,
        mavlog.target_component,
        seq,
        frame,
        cmd,
        1 if is_first else 0,
        1,
        0, 0, 0, 0,
        int(wp['lat'] * 1e7) if 'lat' in wp else 0,
        int(wp['lon'] * 1e7) if 'lon' in wp else 0,
        float(wp['alt']) if 'alt' in wp else 0,
        0
    )
    print(f"Waypoint {seq} - CMD: {cmd} - Lat: {wp.get('lat')}, Lon: {wp.get('lon')}, Alt: {wp.get('alt')}")

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
    alt = home.altitude / 1000.0
    
    print(f"Posición home obtenida: Lat={lat}, Lon={lon}, Alt={alt}")
    return {"lat": lat, "lon": lon, "alt": alt}

def send_mavlink_commands():
    """Solicita envíos de distintos mensajes al autopiloto"""
    mavlog.mav.command_long_send(1, 0, 512, 0, 2, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 27, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 74, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 24, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 36, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 30, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 33, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 136, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 143, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 1, 1e6/50, 0, 0, 0, 0, 0)
    mavlog.mav.command_long_send(1, 0, 512, 0, 147, 1e6/50, 0, 0, 0, 0, 0)

def process_mavlink_message(msg):
    """Procesa mensajes MAVLink y actualiza variables globales"""
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
    
    elif msg.get_type() == 'HEARTBEAT':
        try:
            global flight_mode, armed
            ARDUCOPTER_MODES = {
                0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO',
                4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 7: 'CIRCLE',
                9: 'LAND', 11: 'DRIFT', 13: 'SPORT', 16: 'POSHOLD',
                17: 'BRAKE', 18: 'THROW', 21: 'SMART_RTL'
            }
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            flight_mode = ARDUCOPTER_MODES.get(msg.custom_mode, f'MODE_{msg.custom_mode}')
        except:
            pass

    elif msg.get_type() == 'BATTERY_STATUS':
        try:
            voltage_battery = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else None
            current_battery = msg.current_battery / 100.0 if msg.current_battery != -1 else None
            battery_remaining = msg.battery_remaining if msg.battery_remaining != -1 else None
        except:
            pass

# ============================================================
# NUEVA FUNCIÓN: Configurar FENCE (Geofence) con RTL
# ============================================================
def send_fence(vertices, breach_action='RTL'):
    with mavlink_io_lock:
        _send_fence_locked(vertices, breach_action)

def _send_fence_locked(vertices, breach_action='RTL'):
    print(f"\n{'='*60}")
    print(f"🚧 AÑADIENDO FENCE DE INCLUSIÓN con {len(vertices)} vértices")
    print(f"{'='*60}")

    action_map = {'NONE': 0, 'RTL': 1, 'LAND': 2}
    action_value = action_map.get(breach_action, 1)

    # 1. Leer fences existentes (igual que en exclusion)
    existing = get_existing_fences()

    # 2. Calcular FENCE_TYPE combinado
    has_exclusion = any(
        item['command'] == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        for item in existing
    )
    fence_type = 12 if has_exclusion else 4  # 4+8=12 ambas, 4=solo inclusión

    # 3. Parámetros
    print("1️⃣ Configurando parámetros...")
    params = [
        (b'FENCE_ENABLE', 1),
        (b'FENCE_TYPE', fence_type),
        (b'FENCE_ACTION', action_value),
        (b'FENCE_ALT_MAX', 100),
    ]
    for param_name, param_value in params:
        mavlog.mav.param_set_send(
            mavlog.target_system, mavlog.target_component,
            param_name, param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.3)
        print(f"   ✅ {param_name.decode()} = {param_value}")

    # 4. Limpiar
    print("2️⃣ Limpiando fence anterior...")
    mavlog.mav.mission_clear_all_send(
        mavlog.target_system, mavlog.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    )
    ack = mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    print(f"   ✅ Limpiado")
    time.sleep(0.5)

    # Solo contar exclusiones existentes + nueva inclusión
    existing_exclusions = [i for i in existing if i['command'] == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION]
    total_points = len(existing_exclusions) + len(vertices)
    print(f"3️⃣ Enviando MISSION_COUNT = {total_points} ({len(existing_exclusions)} exclusiones existentes + {len(vertices)} nueva inclusión)...")
    mavlog.mav.mission_count_send(
        mavlog.target_system, mavlog.target_component,
        total_points,
        mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    )

    # 6. Reenviar existentes — SOLO las de exclusión, ignorar inclusiones anteriores
    seq = 0
    for item in existing_exclusions:
        # ❌ Saltar fences de inclusión anteriores
        if item['command'] == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            continue

        req = mavlog.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=5
        )
        if req is None:
            print(f"   ❌ Timeout esperando request para existente seq={seq}")
            return
        if req.get_type() == 'MISSION_ACK':
            print(f"   ❌ MISSION_ACK inesperado (error del autopiloto): type={req.type}")
            return
        if req.seq != seq:
            print(f"   ❌ Secuencia incorrecta: esperado {seq}, recibido {req.seq}")
            return

        mavlog.mav.mission_item_int_send(
            mavlog.target_system, mavlog.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            item['command'],
            0, 1,
            int(item['param1']),
            0, 0, 0,
            int(item['lat'] * 1e7),
            int(item['lon'] * 1e7),
            0,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        print(f"   ♻️ Reenviado existente {seq}: CMD={item['command']}")
        seq += 1

    # 7. Enviar nueva fence de inclusión
    print("4️⃣ Enviando nueva fence de inclusión...")
    for vertex in vertices:
        req = mavlog.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=5
        )
        if req is None:
            print(f"   ❌ Timeout esperando request para vértice seq={seq}")
            return
        if req.get_type() == 'MISSION_ACK':
            print(f"   ❌ MISSION_ACK inesperado (error del autopiloto): type={req.type}")
            return
        if req.seq != seq:
            print(f"   ❌ Secuencia incorrecta: esperado {seq}, recibido {req.seq} (tipo: {req.get_type()})")
            return

        mavlog.mav.mission_item_int_send(
            mavlog.target_system, mavlog.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            0, 1,
            len(vertices),
            0, 0, 0,
            int(vertex['lat'] * 1e7),
            int(vertex['lon'] * 1e7),
            0,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        print(f"   ✅ Vértice {seq}: ({vertex['lat']:.6f}, {vertex['lon']:.6f})")
        seq += 1

    # 8. ACK final
    ack = mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    if ack and getattr(ack, 'type', None) == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("✅ Fence de inclusión añadida correctamente")
    else:
        print(f"⚠️ ACK final: {ack}")
    print(f"{'='*60}")
    
def get_existing_fences():
    """Lee las fences actuales del autopiloto"""
    with mavlink_io_lock:
        print("📖 Leyendo fences existentes...")

        mavlog.mav.mission_request_list_send(
            mavlog.target_system,
            mavlog.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )

        count_msg = mavlog.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        if count_msg is None or count_msg.count == 0:
            print("   📭 No hay fences existentes")
            return []

        print(f"   📦 Hay {count_msg.count} items de fence existentes")
        existing = []

        for i in range(count_msg.count):
            mavlog.mav.mission_request_int_send(
                mavlog.target_system,
                mavlog.target_component,
                i,
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE
            )
            item = mavlog.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
            if item:
                existing.append({
                    'seq': i,
                    'command': item.command,
                    'param1': item.param1,
                    'lat': item.x / 1e7,
                    'lon': item.y / 1e7,
                })
                print(f"   ✅ Item {i}: CMD={item.command}, ({item.x/1e7:.6f}, {item.y/1e7:.6f})")

        # Enviar ACK
        mavlog.mav.mission_ack_send(
            mavlog.target_system,
            mavlog.target_component,
            mavutil.mavlink.MAV_MISSION_ACCEPTED,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )

        return existing


def send_exclusion_fence(zones, breach_action='RTL'):
    with mavlink_io_lock:
        _send_exclusion_fence_locked(zones, breach_action)

def _send_exclusion_fence_locked(zones, breach_action='RTL'):
    print(f"\n{'='*60}")
    print(f"🚫 AÑADIENDO {len(zones)} ZONA(S) DE EXCLUSIÓN")
    print(f"{'='*60}")

    action_map = {'NONE': 0, 'RTL': 1, 'LAND': 2}
    action_value = action_map.get(breach_action, 1)

    # 1. Leer fences existentes
    existing = get_existing_fences()

    # 2. Calcular FENCE_TYPE combinado según lo que ya hay + lo nuevo
    has_inclusion = any(
        item['command'] == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        for item in existing
    )
    # Nuevo tipo: inclusión (4) + exclusión (8) si hay ambas, o solo exclusión (8)
    fence_type = 12 if has_inclusion else 8  # 4+8=12 ambas, 8=solo exclusión

    # 3. Parámetros
    print("1️⃣ Configurando parámetros...")
    params = [
        (b'FENCE_ENABLE', 1),
        (b'FENCE_TYPE', fence_type),
        (b'FENCE_ACTION', action_value),
        (b'FENCE_ALT_MAX', 100),
    ]
    for param_name, param_value in params:
        mavlog.mav.param_set_send(
            mavlog.target_system, mavlog.target_component,
            param_name, param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.3)
        print(f"   ✅ {param_name.decode()} = {param_value}")

    # 4. Limpiar
    print("2️⃣ Limpiando fence anterior...")
    mavlog.mav.mission_clear_all_send(
        mavlog.target_system, mavlog.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    )
    ack = mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    print(f"   ✅ Limpiado")
    time.sleep(0.5)

    # 5. Total = existentes + nuevos
    new_points = sum(len(zone) for zone in zones)
    total_points = len(existing) + new_points
    print(f"3️⃣ Enviando MISSION_COUNT = {total_points} ({len(existing)} existentes + {new_points} nuevos)...")
    mavlog.mav.mission_count_send(
        mavlog.target_system, mavlog.target_component,
        total_points,
        mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    )

    # 6. Reenviar existentes
    seq = 0
    for item in existing:
        req = mavlog.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=5
        )
        if req is None:
            print(f"   ❌ Timeout esperando request para existente seq={seq}")
            return
        if req.get_type() == 'MISSION_ACK':
            print(f"   ❌ MISSION_ACK inesperado (error del autopiloto): type={req.type}")
            return
        if req.seq != seq:
            print(f"   ❌ Secuencia incorrecta: esperado {seq}, recibido {req.seq}")
            return

        mavlog.mav.mission_item_int_send(
            mavlog.target_system, mavlog.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            item['command'],
            0, 1,
            int(item['param1']),
            0, 0, 0,
            int(item['lat'] * 1e7),
            int(item['lon'] * 1e7),
            0,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        print(f"   ♻️ Reenviado existente {seq}: CMD={item['command']}")
        seq += 1

    # 7. Enviar nuevas zonas de exclusión
    print("4️⃣ Enviando nuevas zonas de exclusión...")
    for zone_idx, zone in enumerate(zones):
        print(f"   🔴 Zona {zone_idx + 1}: {len(zone)} vértices")
        for vertex in zone:
            req = mavlog.recv_match(
                type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
                blocking=True, timeout=5
            )
            if req is None:
                print(f"   ❌ Timeout esperando request para vértice seq={seq}")
                return
            if req.get_type() == 'MISSION_ACK':
                print(f"   ❌ MISSION_ACK inesperado (error del autopiloto): type={req.type}")
                return
            if req.seq != seq:
                print(f"   ❌ Secuencia incorrecta: esperado {seq}, recibido {req.seq} (tipo: {req.get_type()})")
                return

            mavlog.mav.mission_item_int_send(
                mavlog.target_system, mavlog.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                0, 1,
                len(zone),
                0, 0, 0,
                int(vertex['lat'] * 1e7),
                int(vertex['lon'] * 1e7),
                0,
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE
            )
            print(f"      ✅ Vértice {seq}: ({vertex['lat']:.6f}, {vertex['lon']:.6f})")
            seq += 1

    # 8. ACK final
    ack = mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    if ack and getattr(ack, 'type', None) == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("✅ Zonas de exclusión añadidas correctamente")
    else:
        print(f"⚠️ ACK final: {ack}")
    print(f"{'='*60}")

# ============================================================
# FUNCIONES PARA COMANDOS DE EMERGENCIA
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
                1, 0, 0, 0, 0, 0, 0
            )
            print("✅ Comando ARM enviado")
            
        elif action == 'TAKEOFF':
            print("🛫 DESPEGANDO a 20m...")
            mavlog.mav.command_long_send(
                mavlog.target_system, mavlog.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, 20
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
                5, 0, 0, 0, 0, 0
            )
        
        print(f"✅ Comando {action} ejecutado correctamente")
        
    except Exception as e:
        print(f"❌ Error en comando {action}: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        with emergencia_lock:
            emergencia_en_curso = False

# ============================================================
# PROCESAMIENTO DE COMANDOS
# ============================================================

def process_command(json_data):
    """
    Ejecuta la lógica de comandos recibidos por MQTT
    """
    try:
        print(f"\n📨 Comando recibido para procesar: {json_data}")

        action = json_data.get('action')
        new_lat = json_data.get('new_lat')
        new_lon = json_data.get('new_lon')
        new_alt = json_data.get('new_alt')
        waypoints = json_data.get('waypoints', [])
        vertices = json_data.get('vertices', [])
        action_on_break = json_data.get('action_on_break', 'RTL')
        
        # ============================================================
        # COMANDO FENCE
        # ============================================================
        if action == 'FENCE' and vertices and len(vertices) >= 3:
            print(f"🚧 Configurando FENCE con {len(vertices)} vértices")
            print(f"🚧 Acción al romper: {action_on_break}")
            send_fence(vertices, action_on_break)
            print("✅ FENCE configurado correctamente")
            return
        
        elif action == 'EXCLUSION_FENCE' and vertices and len(vertices) >= 1:
            # vertices aquí es una lista de zonas: [[{lat,lon},...], [{lat,lon},...]]
            # Si viene como lista plana de una sola zona, la envolvemos
            if isinstance(vertices[0], dict):
                zones = [vertices]  # una sola zona
            else:
                zones = vertices    # múltiples zonas
            
            print(f"🚫 Configurando {len(zones)} zona(s) de exclusión")
            send_exclusion_fence(zones, action_on_break)
            return

        elif action == 'CLEAR_FENCES':
            # Borra TODO (inclusión + exclusión)
            print("🗑️ Borrando todas las fences...")
            with mavlink_io_lock:
                mavlog.mav.mission_clear_all_send(
                    mavlog.target_system, mavlog.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE
                )
                mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
                mavlog.mav.param_set_send(
                    mavlog.target_system, mavlog.target_component,
                    b'FENCE_ENABLE', 0,
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )
            print("✅ Todas las fences borradas")
            return

        elif action == 'CLEAR_EXCLUSION_FENCES':
            # Lee existentes, descarta exclusiones, reenvía solo inclusiones
            print("🗑️ Borrando solo fences de exclusión...")
            with mavlink_io_lock:
                existing = get_existing_fences()

                inclusions = [i for i in existing
                            if i['command'] == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION]

                # Limpiar todo
                mavlog.mav.mission_clear_all_send(
                    mavlog.target_system, mavlog.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE
                )
                mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
                time.sleep(0.5)

                if len(inclusions) == 0:
                    # No hay inclusiones que preservar, desactivar fence
                    mavlog.mav.param_set_send(
                        mavlog.target_system, mavlog.target_component,
                        b'FENCE_ENABLE', 0,
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                    )
                    print("✅ Exclusiones borradas, no había inclusiones, fence desactivada")
                    return

                # Reenviar solo las inclusiones
                mavlog.mav.param_set_send(
                    mavlog.target_system, mavlog.target_component,
                    b'FENCE_TYPE', 4,  # solo inclusión
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )
                time.sleep(0.3)

                mavlog.mav.mission_count_send(
                    mavlog.target_system, mavlog.target_component,
                    len(inclusions),
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE
                )

                for seq, item in enumerate(inclusions):
                    req = mavlog.recv_match(
                        type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                        blocking=True, timeout=5
                    )
                    if req is None or req.seq != seq:
                        print(f"   ❌ Error en secuencia {seq}")
                        return
                    mavlog.mav.mission_item_int_send(
                        mavlog.target_system, mavlog.target_component,
                        seq,
                        mavutil.mavlink.MAV_FRAME_GLOBAL,
                        item['command'],
                        0, 1,
                        int(item['param1']),
                        0, 0, 0,
                        int(item['lat'] * 1e7),
                        int(item['lon'] * 1e7),
                        0,
                        mavutil.mavlink.MAV_MISSION_TYPE_FENCE
                    )
                    print(f"   ♻️ Inclusión reenviada seq={seq}")

                ack = mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
                if ack and getattr(ack, 'type', None) == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("✅ Exclusiones borradas, inclusiones preservadas")
                else:
                    print(f"⚠️ ACK final: {ack}")
            return

        # ============================================================
        # COMANDO AUTO (MISIÓN)
        # ============================================================
        elif action == 'AUTO':
            print("🔄 Cambiando a modo GUIDED (preparando misión AUTO)...")
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
                print(f"❌ Error al cambiar a modo GUIDED: {ack}")
                return
            print("✅ Modo GUIDED establecido")
            time.sleep(2)
            
            print("🗑️ Limpiando misión actual...")
            mavlog.mav.mission_clear_all_send(mavlog.target_system, mavlog.target_component)
            ack = mavlog.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
            if ack is None or getattr(ack, 'type', None) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f"❌ Error al limpiar la misión: {ack}")
                return
            print("✅ Misión actual limpiada")
            
            home = get_home_position()
            if home is None:
                return
            
            takeoff_alt = waypoints[0]['alt'] if waypoints else 50
            takeoff_cmd = {'lat': home['lat'], 'lon': home['lon'], 'alt': takeoff_alt}
            all_waypoints = [home, takeoff_cmd] + waypoints

            print(f"📦 Enviando misión con {len(all_waypoints)} comandos")
            mavlog.mav.mission_count_send(mavlog.target_system, mavlog.target_component, len(all_waypoints))

            for i, wp in enumerate(all_waypoints):
                req = mavlog.recv_match(type=['MISSION_REQUEST','MISSION_REQUEST_INT'], blocking=True, timeout=5)
                if not req or req.seq != i:
                    print(f"❌ Error en secuencia: esperado {i}, recibido {getattr(req,'seq',None)}")
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
                print(f"❌ Error en confirmación final: {ack}")
                return
            print("✅ Misión configurada correctamente")

            print("🔓 Armando el dron...")
            mavlog.mav.command_long_send(mavlog.target_system, mavlog.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                         0, 1, 0, 0, 0, 0, 0, 0)
            time.sleep(2)

            print("🔄 Cambiando a modo AUTO...")
            mavlog.mav.command_long_send(mavlog.target_system, mavlog.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                         0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3, 0, 0, 0, 0, 0)
            ack = mavlog.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack is None or getattr(ack, 'result', None) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"❌ Error al cambiar a modo AUTO: {ack}")
                return
            print("✅ Modo AUTO establecido")

            print("▶️ Iniciando la misión...")
            mavlog.mav.command_long_send(mavlog.target_system, mavlog.target_component,
                                         mavutil.mavlink.MAV_CMD_MISSION_START,
                                         0, 0, 0, 0, 0, 0, 0, 0)
            ack = mavlog.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack is None or getattr(ack, 'result', None) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"❌ Error al iniciar la misión: {ack}")
                return
            print("✅ Misión iniciada correctamente")

        # ============================================================
        # COMANDO GUIDED (IR A PUNTO)
        # ============================================================
        elif action == 'GUIDED' and new_lat is not None and new_lon is not None and new_alt is not None:
            print(f"🎯 Enviando comando GUIDED a ({new_lat}, {new_lon}, {new_alt})")
            
            mavlog.mav.command_long_send(
                mavlog.target_system,
                mavlog.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4,
                0, 0, 0, 0, 0
            )
            time.sleep(1)

            mavlog.mav.command_int_send(
                mavlog.target_system,
                mavlog.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0,
                0,
                0,
                1,
                0,
                0,
                int(float(new_lat) * 1e7),
                int(float(new_lon) * 1e7),
                float(new_alt)
            )
            
            ack = mavlog.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack is None or getattr(ack, 'result', None) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"❌ Error al enviar comando GUIDED: {ack}")
                return
            
            print("✅ Comando GUIDED enviado correctamente")

    except Exception as e:
        print(f"❌ Error procesando comando: {e}")
        import traceback
        traceback.print_exc()

# ============================================================
# PUBLICACIÓN DE TELEMETRÍA MQTT
# ============================================================

def mqtt_publisher(broker_pub, port_pub, topic_pub, broker_sub, port_sub, topic_sub):
    """
    Publica telemetría periódicamente al broker MQTT
    """
    global latitude, longitude, altitude_agl, altitude_ahl, altitude_asl, groundspeed, airspeed, heading, pitch, yaw, roll, gps_timestamp, system_time, attitude_time, global_position_time, voltage_battery, current_battery, battery_remaining, flight_mode, armed
    
    client = paho.Client(client_id='SIR_Client', transport='websockets', callback_api_version=paho.CallbackAPIVersion.VERSION1)
    client.username_pw_set('bcn', 'Barcelona_1234')
    context = ssl.SSLContext(protocol=ssl.PROTOCOL_TLSv1_2)
    client.tls_set_context(context)
    client.ws_set_options(path="/mqtt/")

    print(f"🔌 Conectando al broker MQTT: {broker_pub}")
    try:
        client.connect(broker_pub, port_pub)
        client.loop_start()
        print(f"✅ Cliente MQTT de telemetría activo. Publicando en: {topic_pub}")
    except Exception as e:
        print(f"❌ Error conectando al broker MQTT: {e}")
        return
    
    while True:
        try:
            # send_mavlink_commands también debe ceder cuando hay un comando activo
            if mavlink_io_lock.acquire(blocking=False):
                try:
                    send_mavlink_commands()
                finally:
                    mavlink_io_lock.release()

            while True:
                # Si hay una operación de fence/comando activa, ceder el acceso
                if not mavlink_io_lock.acquire(blocking=False):
                    time.sleep(0.005)
                    break
                try:
                    msg = mavlog.recv_match(
                        type=['SYSTEM_TIME', 'RAW_IMU', 'VFR_HUD', 'GPS_RAW_INT',
                            'SERVO_OUTPUT_RAW', 'ATTITUDE', 'GLOBAL_POSITION_INT',
                            'TERRAIN_REPORT', 'SCALED_PRESSURE3', 'SYS_STATUS',
                            'HEARTBEAT', 'BATTERY_STATUS'],
                        blocking=True,
                        timeout=0.02
                    )
                finally:
                    mavlink_io_lock.release()
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
                'flight_mode': flight_mode,   # ← añade
                'armed': armed,
                'drone_id': f"{mavlog.target_system}_{mavlog.target_component}"
            }
            payload = json.dumps(msg, indent=4)
            client.publish(topic_pub, str(payload), qos=0)
            
            time.sleep(0.1)
            
        except Exception as e:
            print(f"❌ Error en el bucle de telemetría: {e}")
            import traceback
            traceback.print_exc()

# ============================================================
# CONEXIÓN MAVLINK
# ============================================================

def connection():
    global mavlog
    import sys
    import platform
    
    sistema = platform.system()
    
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
        if sistema == "Windows":
            print("Windows detectado - Usando TCP (modo prueba)")
            mavlog = mavutil.mavlink_connection('tcp:127.0.0.1:5762', wait_ready=True, baud=115200)
        else:
            print("Linux detectado - Usando serie /dev/serial0")
            mavlog = mavutil.mavlink_connection('/dev/serial0', wait_ready=True, baud=57600)
    
    time.sleep(5)
    print('✅ Conectado al autopiloto')

# ============================================================
# ENDPOINTS FLASK
# ============================================================

@app.route("/action", methods=["POST"])
def action_handler():
    try:
        json_data = request.get_json(force=True)
        if not json_data:
            return "No JSON recibido", 400

        if "msg" in json_data:
            try:
                msg = json.loads(json_data["msg"])
            except Exception as e:
                print(f"❌ Error parseando msg: {e}")
                return "Bad msg JSON", 400
        else:
            msg = json_data

        action = msg.get('action')
        
        acciones_emergencia = ['ARM', 'TAKEOFF', 'DISARM', 'EMERGENCY_STOP', 'RTL', 'LAND_NOW', 'HOLD_POSITION']
        acciones_normales = ['AUTO', 'GUIDED', 'FENCE', 'EXCLUSION_FENCE', 'CLEAR_FENCES', 'CLEAR_EXCLUSION_FENCES']
        
        if action in acciones_emergencia:
            print(f"🚨 Comando de emergencia: {action}")
            t = threading.Thread(target=ejecutar_emergencia, args=(action,))
            t.daemon = True
            t.start()
            return "EMERGENCY EXECUTED", 200
        elif action in acciones_normales:
            print(f"📋 Comando normal: {action}")
            t = threading.Thread(target=process_command, args=(msg,))
            t.daemon = True
            t.start()
            return "OK", 200
        else:
            print(f"⚠️ Acción desconocida: {action}")
            return "Unknown action", 400
            
    except Exception as e:
        print(f"❌ Error en endpoint /action: {e}")
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
        print(f"❌ Error en emergencia: {e}")
        return "Error", 500

@app.route("/health", methods=["GET"])
def health_check():
    """Endpoint para verificar que el servicio está vivo"""
    return {"status": "ok", "connected": mavlog is not None}, 200

# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    broker = 'mqtt.catuav.com'
    port = 443
    topic_pub = 'drone_aerowatch_telemetry'
    topic_sub = 'drone_aerowatch_action'

    print("=" * 60)
    print("🚁 DRONE MAVLINK BRIDGE - SCRIPT COMPLETO")
    print("=" * 60)
    print("\nModos de uso:")
    print("  python mavlink.py           -> Auto-detección")
    print("  python mavlink.py tcp       -> Conexión TCP (127.0.0.1:5762)")
    print("  python mavlink.py udp       -> Conexión UDP (127.0.0.1:14550)")
    print("  python mavlink.py serial    -> Conexión serie (/dev/serial0)")
    print("  python mavlink.py /dev/ttyUSB0 -> Puerto específico")
    print()

    connection()

    print("\n📡 Iniciando cliente MQTT (telemetría)...")
    mqtt_thread = threading.Thread(target=mqtt_publisher, args=(broker, port, topic_pub, broker, port, topic_sub))
    mqtt_thread.daemon = True
    mqtt_thread.start()


    print("\n🌐 Levantando servidor HTTP en puerto 5000")
    print("📡 Endpoints disponibles:")
    print("   POST /action    - Comandos normales (AUTO, GUIDED, FENCE)")
    print("   POST /emergency - Comandos de emergencia (ARM, TAKEOFF, RTL, etc.)")
    print("   GET  /health    - Health check")
    print("\nComandos soportados:")
    print("   AUTO     - Misión con waypoints")
    print("   GUIDED   - Movimiento a punto específico")
    print("   FENCE    - Configurar geofence poligonal (con RTL)")
    print("   ARM      - Armar dron")
    print("   TAKEOFF  - Despegar a 20m")
    print("   DISARM   - Desarmar dron")
    print("   RTL      - Return To Launch")
    print("   LAND_NOW - Aterrizar inmediatamente")
    print("   HOLD_POSITION - Mantener posición")
    print("   EMERGENCY_STOP - Parada de emergencia")
    print("=" * 60)
    
    app.run(host="0.0.0.0", port=5000, threaded=True)