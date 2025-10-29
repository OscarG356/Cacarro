#!/usr/bin/env python3
"""
Visualizador de Odometría SIN dependencias de pyserial
Solución para el error "ModuleNotFoundError: No module named 'serial'"
"""

import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import animation
from collections import deque
import threading
import re

class OdometriaVisualizerSinSerial:
    """
    Visualizador que no requiere pyserial - lee datos desde archivo o entrada manual
    """
    
    def __init__(self, buffer_size=None):
        self.buffer_size = buffer_size
        
        # Buffers para almacenar datos - SIN LÍMITE para mantener trayectoria completa
        self.x_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        self.y_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        self.theta_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        self.time_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        
        # Variables para los datos actuales
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Control de threads
        self.running = True
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
        
    def parse_data_line(self, line):
        """Parsea una línea de datos de odometría"""
        try:
            line = line.strip()
            
            # Formato x:val,y:val,theta:val
            pattern = r'x\s*:\s*([+-]?\d*\.?\d+)\s*,\s*y\s*:\s*([+-]?\d*\.?\d+)\s*,\s*theta\s*:\s*([+-]?\d*\.?\d+)'
            match = re.search(pattern, line, re.IGNORECASE)
            
            if match:
                return float(match.group(1)), float(match.group(2)), float(match.group(3))
            
            # Formato separado por comas
            parts = line.split(',')
            if len(parts) >= 3:
                return float(parts[0].strip()), float(parts[1].strip()), float(parts[2].strip())
                
        except (ValueError, IndexError):
            pass
        
        return None
    
    def read_data_from_file(self, filename):
        """Lee datos desde un archivo"""
        print(f"Leyendo datos desde: {filename}")
        
        try:
            with open(filename, 'r') as f:
                for line in f:
                    if not self.running:
                        break
                        
                    parsed = self.parse_data_line(line)
                    if parsed:
                        x, y, theta = parsed
                        self.add_data_point(x, y, theta)
                        time.sleep(0.1)  # Simular tiempo real
        except FileNotFoundError:
            print(f"Archivo {filename} no encontrado. Usando datos simulados...")
            self.generate_simulated_data()
        except Exception as e:
            print(f"Error leyendo archivo: {e}. Usando datos simulados...")
            self.generate_simulated_data()
    
    def generate_simulated_data(self):
        """Genera datos simulados para demostración"""
        print("Generando datos simulados...")
        
        x, y, theta = 0.0, 0.0, 0.0
        t = 0.0
        
        while self.running:
            dt = 0.1
            
            # Diferentes patrones de movimiento
            if t < 5.0:
                v, w = 2.0, 0.0  # Recto
            elif t < 8.0:
                v, w = 0.0, 1.0  # Giro
            elif t < 13.0:
                v, w = 1.5, 0.0  # Recto otra vez
                theta = math.pi / 2  # Hacia arriba
            elif t < 16.0:
                v, w = 0.0, -1.2  # Giro contrario
            elif t < 21.0:
                v, w = 1.8, 0.0  # Diagonal
                theta = -math.pi / 4
            else:
                v, w = 1.2, 0.5  # Círculo
            
            # Actualizar posición
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            theta = ((theta + math.pi) % (2 * math.pi)) - math.pi
            
            self.add_data_point(x, y, theta)
            
            t += dt
            if t > 30.0:  # Reiniciar ciclo
                t = 0.0
                x, y, theta = 0.0, 0.0, 0.0
            
            time.sleep(0.1)
    
    def add_data_point(self, x, y, theta):
        """Agrega un punto de datos al buffer"""
        current_time = time.time()
        
        with self.data_lock:
            self.current_x = x
            self.current_y = y
            self.current_theta = theta
            
            self.x_data.append(x)
            self.y_data.append(y)
            self.theta_data.append(theta)
            self.time_data.append(current_time)
            
            # Actualizar límites dinámicos
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
        self.fig.suptitle('Visualizador de Odometría (Sin pyserial)', fontsize=16, fontweight='bold')
        
        # Gráfica principal: Trayectoria con cono de visión
        self.ax1.set_title('Trayectoria con Ejes Dinámicos', fontweight='bold')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axis('equal')
        
        self.trajectory_line, = self.ax1.plot([], [], 'navy', linewidth=3, alpha=0.8, label='Trayectoria')
        self.robot_point, = self.ax1.plot([], [], 'ro', markersize=12, markeredgecolor='black', markeredgewidth=2, label='Robot')
        
        # Cono de visión
        self.vision_cone = Polygon([(0, 0)], closed=True, alpha=0.5, facecolor='orange', edgecolor='red', linewidth=2)
        self.ax1.add_patch(self.vision_cone)
        
        # Flecha de dirección
        self.direction_line, = self.ax1.plot([], [], 'red', linewidth=4, alpha=0.9, label='Dirección')
        
        # Información
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
                
                # *** EJES DINÁMICOS ***
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
                    
                    limits_text = f"Área: {x_range:.1f}m × {y_range:.1f}m\n"
                    limits_text += f"X: [{self.x_min_ever:.1f}, {self.x_max_ever:.1f}]\n"
                    limits_text += f"Y: [{self.y_min_ever:.1f}, {self.y_max_ever:.1f}]"
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
    
    def start_visualization(self, data_source='simulated'):
        """Inicia la visualización"""
        print("=== VISUALIZADOR SIN PYSERIAL ===")
        print("Opciones de fuente de datos:")
        print("1. 'simulated' - Datos simulados")
        print("2. 'archivo.txt' - Lee desde archivo")
        print("3. 'manual' - Entrada manual")
        print()
        
        self.setup_plots()
        
        # Iniciar fuente de datos en hilo separado
        if data_source == 'simulated':
            data_thread = threading.Thread(target=self.generate_simulated_data, daemon=True)
        elif data_source.endswith('.txt'):
            data_thread = threading.Thread(target=self.read_data_from_file, args=(data_source,), daemon=True)
        else:
            data_thread = threading.Thread(target=self.manual_data_input, daemon=True)
        
        data_thread.start()
        
        # Configurar animación
        ani = animation.FuncAnimation(self.fig, self.update_plots, interval=100, blit=False)
        
        plt.tight_layout()
        plt.show()
    
    def manual_data_input(self):
        """Permite entrada manual de datos"""
        print("Entrada manual de datos. Formato: x,y,theta")
        print("Ejemplo: 1.5,2.3,0.785")
        print("Escribe 'quit' para salir")
        
        while self.running:
            try:
                line = input("Datos (x,y,theta): ")
                if line.lower() == 'quit':
                    break
                
                parsed = self.parse_data_line(line)
                if parsed:
                    x, y, theta = parsed
                    self.add_data_point(x, y, theta)
                    print(f"✓ Agregado: ({x:.2f}, {y:.2f}, {theta:.2f})")
                else:
                    print("✗ Formato incorrecto. Use: x,y,theta")
            except (EOFError, KeyboardInterrupt):
                break
    
    def stop(self):
        """Detiene la visualización"""
        self.running = False
        plt.close('all')

def main():
    visualizer = OdometriaVisualizerSinSerial()
    
    print("¿Qué fuente de datos quieres usar?")
    print("1. Datos simulados (recomendado)")
    print("2. Leer desde archivo")
    print("3. Entrada manual")
    
    try:
        choice = input("Opción (1-3): ").strip()
        
        if choice == '2':
            filename = input("Nombre del archivo: ").strip()
            visualizer.start_visualization(filename)
        elif choice == '3':
            visualizer.start_visualization('manual')
        else:
            visualizer.start_visualization('simulated')
            
    except KeyboardInterrupt:
        print("\nVisualizador detenido")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        visualizer.stop()

if __name__ == "__main__":
    main()