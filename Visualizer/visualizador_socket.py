#!/usr/bin/env python3
"""
Visualizador de Odometría por SOCKET
Alternativa a pyserial - recibe datos por red TCP
"""

import socket
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import animation
from collections import deque
import threading
import re
import json

class OdometriaVisualizerSocket:
    """
    Visualizador que recibe datos por socket TCP
    """
    
    def __init__(self, host='localhost', port=8888, buffer_size=None):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        
        # Socket
        self.socket = None
        self.client_socket = None
        
        # Buffers - SIN LÍMITE para mantener trayectoria completa
        self.x_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        self.y_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        self.theta_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        self.time_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        
        # Variables actuales
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Control
        self.running = True
        self.connected = False
        self.data_lock = threading.Lock()
        
        # Límites dinámicos
        self.x_min_ever = float('inf')
        self.x_max_ever = float('-inf')
        self.y_min_ever = float('inf')
        self.y_max_ever = float('-inf')
        self.limits_initialized = False
        
        # Configuración visual
        self.cone_length = 2.5
        self.cone_angle = 0.4
        
    def create_server(self):
        """Crea servidor TCP para recibir datos"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(1)
            
            print(f"🌐 Servidor iniciado en {self.host}:{self.port}")
            print(f"💡 Esperando conexión...")
            print(f"📝 Para enviar datos, conecta a: telnet {self.host} {self.port}")
            print(f"📋 Formato: x:1.5,y:2.3,theta:0.785")
            print()
            
            return True
        except Exception as e:
            print(f"❌ Error creando servidor: {e}")
            return False
    
    def wait_for_connection(self):
        """Espera conexión de cliente"""
        try:
            while self.running:
                self.socket.settimeout(1.0)
                try:
                    self.client_socket, addr = self.socket.accept()
                    self.connected = True
                    print(f"✅ Cliente conectado desde: {addr}")
                    return True
                except socket.timeout:
                    continue
        except Exception as e:
            print(f"❌ Error esperando conexión: {e}")
        return False
    
    def parse_data_line(self, line):
        """Parsea línea de datos"""
        try:
            line = line.strip()
            
            # Formato JSON
            if line.startswith('{'):
                data = json.loads(line)
                return float(data['x']), float(data['y']), float(data['theta'])
            
            # Formato x:val,y:val,theta:val
            pattern = r'x\s*:\s*([+-]?\d*\.?\d+)\s*,\s*y\s*:\s*([+-]?\d*\.?\d+)\s*,\s*theta\s*:\s*([+-]?\d*\.?\d+)'
            match = re.search(pattern, line, re.IGNORECASE)
            
            if match:
                return float(match.group(1)), float(match.group(2)), float(match.group(3))
            
            # Formato separado por comas
            parts = line.split(',')
            if len(parts) >= 3:
                return float(parts[0].strip()), float(parts[1].strip()), float(parts[2].strip())
                
        except (ValueError, IndexError, json.JSONDecodeError):
            pass
        
        return None
    
    def read_socket_data(self):
        """Lee datos del socket"""
        buffer = ""
        
        while self.running and self.connected:
            try:
                self.client_socket.settimeout(1.0)
                data = self.client_socket.recv(1024).decode('utf-8')
                
                if not data:
                    print("🔌 Cliente desconectado")
                    self.connected = False
                    break
                
                buffer += data
                lines = buffer.split('\n')
                buffer = lines[-1]  # Mantener línea incompleta
                
                for line in lines[:-1]:
                    if line.strip():
                        parsed = self.parse_data_line(line)
                        if parsed:
                            x, y, theta = parsed
                            self.add_data_point(x, y, theta)
                            print(f"📍 Recibido: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")
                        else:
                            print(f"⚠️  Formato incorrecto: {line}")
                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"❌ Error leyendo socket: {e}")
                self.connected = False
                break
    
    def add_data_point(self, x, y, theta):
        """Agrega punto de datos"""
        current_time = time.time()
        
        with self.data_lock:
            self.current_x = x
            self.current_y = y
            self.current_theta = theta
            
            self.x_data.append(x)
            self.y_data.append(y)
            self.theta_data.append(theta)
            self.time_data.append(current_time)
            
            # Actualizar límites
            if not self.limits_initialized:
                self.x_min_ever = self.x_max_ever = x
                self.y_min_ever = self.y_max_ever = y
                self.limits_initialized = True
            else:
                self.x_min_ever = min(self.x_min_ever, x)
                self.x_max_ever = max(self.x_max_ever, x)
                self.y_min_ever = min(self.y_min_ever, y)
                self.y_max_ever = max(self.y_max_ever, y)
    
    def calculate_cone(self, x, y, theta):
        """Calcula puntos del cono de visión"""
        tip_x = x + self.cone_length * math.cos(theta)
        tip_y = y + self.cone_length * math.sin(theta)
        
        left_x = x + self.cone_length * math.cos(theta + self.cone_angle)
        left_y = y + self.cone_length * math.sin(theta + self.cone_angle)
        
        right_x = x + self.cone_length * math.cos(theta - self.cone_angle)
        right_y = y + self.cone_length * math.sin(theta - self.cone_angle)
        
        return [(x, y), (left_x, left_y), (tip_x, tip_y), (right_x, right_y)]
    
    def setup_plots(self):
        """Configura las gráficas"""
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle(f'Visualizador por Socket - {self.host}:{self.port}', fontsize=16, fontweight='bold')
        
        # Gráfica principal
        self.ax1.set_title('Trayectoria con Ejes Dinámicos')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axis('equal')
        
        self.trajectory_line, = self.ax1.plot([], [], 'navy', linewidth=3, alpha=0.8, label='Trayectoria')
        self.robot_point, = self.ax1.plot([], [], 'ro', markersize=12, markeredgecolor='black', markeredgewidth=2, label='Robot')
        
        self.vision_cone = Polygon([(0, 0)], closed=True, alpha=0.5, facecolor='orange', edgecolor='red', linewidth=2)
        self.ax1.add_patch(self.vision_cone)
        
        self.direction_line, = self.ax1.plot([], [], 'red', linewidth=4, alpha=0.9, label='Dirección')
        
        # Textos informativos
        self.connection_text = self.ax1.text(0.98, 0.98, '', transform=self.ax1.transAxes, fontsize=10,
                                           verticalalignment='top', horizontalalignment='right',
                                           bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
        
        self.info_text = self.ax1.text(0.02, 0.98, '', transform=self.ax1.transAxes, fontsize=10,
                                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        self.limits_info = self.ax1.text(0.02, 0.02, '', transform=self.ax1.transAxes, fontsize=9,
                                        verticalalignment='bottom', bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        
        self.ax1.legend()
        
        # Otras gráficas
        self.ax2.set_title('Posición X vs Tiempo')
        self.ax2.set_xlabel('Tiempo (s)')
        self.ax2.set_ylabel('X (m)')
        self.ax2.grid(True)
        self.x_time_line, = self.ax2.plot([], [], 'g-', linewidth=2)
        
        self.ax3.set_title('Posición Y vs Tiempo')
        self.ax3.set_xlabel('Tiempo (s)')
        self.ax3.set_ylabel('Y (m)')
        self.ax3.grid(True)
        self.y_time_line, = self.ax3.plot([], [], 'r-', linewidth=2)
        
        self.ax4.set_title('Orientación Theta vs Tiempo')
        self.ax4.set_xlabel('Tiempo (s)')
        self.ax4.set_ylabel('Theta (rad)')
        self.ax4.grid(True)
        self.theta_time_line, = self.ax4.plot([], [], 'm-', linewidth=2)
    
    def update_plots(self, frame):
        """Actualiza las gráficas"""
        # Estado de conexión
        if self.connected:
            self.connection_text.set_text("🟢 CONECTADO")
            self.connection_text.set_bbox(dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
        else:
            self.connection_text.set_text("🔴 DESCONECTADO")
            self.connection_text.set_bbox(dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
        
        with self.data_lock:
            if len(self.x_data) > 0:
                # Trayectoria y robot
                self.trajectory_line.set_data(list(self.x_data), list(self.y_data))
                self.robot_point.set_data([self.current_x], [self.current_y])
                
                # Cono de visión
                cone_points = self.calculate_cone(self.current_x, self.current_y, self.current_theta)
                self.vision_cone.set_xy(cone_points)
                
                # Flecha direccional
                arrow_end_x = self.current_x + 2.0 * math.cos(self.current_theta)
                arrow_end_y = self.current_y + 2.0 * math.sin(self.current_theta)
                self.direction_line.set_data([self.current_x, arrow_end_x], [self.current_y, arrow_end_y])
                
                # Ejes dinámicos
                if self.limits_initialized:
                    x_range = max(self.x_max_ever - self.x_min_ever, 0.1)
                    y_range = max(self.y_max_ever - self.y_min_ever, 0.1)
                    
                    margin_x = max(x_range * 0.15, 2.0)
                    margin_y = max(y_range * 0.15, 2.0)
                    
                    new_x_min = self.x_min_ever - margin_x
                    new_x_max = self.x_max_ever + margin_x
                    new_y_min = self.y_min_ever - margin_y
                    new_y_max = self.y_max_ever + margin_y
                    
                    current_xlim = self.ax1.get_xlim()
                    current_ylim = self.ax1.get_ylim()
                    
                    final_x_min = min(current_xlim[0], new_x_min)
                    final_x_max = max(current_xlim[1], new_x_max)
                    final_y_min = min(current_ylim[0], new_y_min)
                    final_y_max = max(current_ylim[1], new_y_max)
                    
                    self.ax1.set_xlim(final_x_min, final_x_max)
                    self.ax1.set_ylim(final_y_min, final_y_max)
                    
                    # Información
                    info = f"Posición: ({self.current_x:.1f}, {self.current_y:.1f})\n"
                    info += f"Orientación: {math.degrees(self.current_theta):.0f}°\n"
                    info += f"Puntos: {len(self.x_data)}"
                    self.info_text.set_text(info)
                    
                    limits_text = f"Puerto: {self.port}\n"
                    limits_text += f"Área: {x_range:.1f}m × {y_range:.1f}m"
                    self.limits_info.set_text(limits_text)
                
                # Gráficas temporales
                if len(self.time_data) > 0:
                    start_time = self.time_data[0]
                    relative_time = [t - start_time for t in self.time_data]
                    
                    self.x_time_line.set_data(relative_time, list(self.x_data))
                    self.y_time_line.set_data(relative_time, list(self.y_data))
                    self.theta_time_line.set_data(relative_time, list(self.theta_data))
                    
                    # Ajustar límites temporales
                    if len(relative_time) > 0:
                        time_min, time_max = min(relative_time), max(relative_time)
                        
                        for ax, data in [(self.ax2, self.x_data), (self.ax3, self.y_data), (self.ax4, self.theta_data)]:
                            if len(data) > 0:
                                data_min, data_max = min(data), max(data)
                                data_margin = (data_max - data_min) * 0.1 or 0.1
                                
                                ax.set_xlim(time_min, time_max + 1)
                                ax.set_ylim(data_min - data_margin, data_max + data_margin)
        
        return (self.trajectory_line, self.robot_point, self.vision_cone, self.direction_line,
                self.x_time_line, self.y_time_line, self.theta_time_line)
    
    def start_visualization(self):
        """Inicia la visualización"""
        if not self.create_server():
            return
        
        self.setup_plots()
        
        # Hilo para manejar conexiones
        def connection_handler():
            while self.running:
                if self.wait_for_connection():
                    self.read_socket_data()
                if not self.running:
                    break
        
        connection_thread = threading.Thread(target=connection_handler, daemon=True)
        connection_thread.start()
        
        # Animación
        ani = animation.FuncAnimation(self.fig, self.update_plots, interval=100, blit=False)
        
        plt.tight_layout()
        
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
    
    def stop(self):
        """Detiene el visualizador"""
        self.running = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        plt.close('all')

def create_test_client(host='localhost', port=8888):
    """Crea cliente de prueba para enviar datos"""
    print(f"🔧 Creando cliente de prueba para {host}:{port}")
    
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect((host, port))
        print("✅ Conectado al servidor")
        
        # Enviar datos de prueba
        x, y, theta = 0.0, 0.0, 0.0
        
        for i in range(100):
            # Movimiento circular
            t = i * 0.1
            x = 5 * math.cos(t * 0.2)
            y = 5 * math.sin(t * 0.2)
            theta = t * 0.2
            
            data = f"x:{x:.2f},y:{y:.2f},theta:{theta:.2f}\n"
            client.send(data.encode('utf-8'))
            print(f"📤 Enviado: {data.strip()}")
            
            time.sleep(0.2)
        
        client.close()
        print("🔚 Cliente terminado")
        
    except Exception as e:
        print(f"❌ Error en cliente: {e}")

def main():
    print("=== VISUALIZADOR POR SOCKET ===")
    print("1. Iniciar servidor (recibir datos)")
    print("2. Cliente de prueba (enviar datos)")
    
    choice = input("Opción (1-2): ").strip()
    
    if choice == '2':
        host = input("Host (localhost): ").strip() or 'localhost'
        port = int(input("Puerto (8888): ").strip() or '8888')
        create_test_client(host, port)
    else:
        host = input("Host (localhost): ").strip() or 'localhost'
        port = int(input("Puerto (8888): ").strip() or '8888')
        
        visualizer = OdometriaVisualizerSocket(host, port)
        
        try:
            visualizer.start_visualization()
        except KeyboardInterrupt:
            print("\nVisualizador detenido")
        finally:
            visualizer.stop()

if __name__ == "__main__":
    main()