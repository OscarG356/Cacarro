#!/usr/bin/env python3
"""
Demo: Trayectoria Completa - Nunca se borra
Demuestra que TODOS los puntos se mantienen visibles
"""

import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import animation
from collections import deque
import threading

class TrayectoriaCompletaDemo:
    """
    Demo que muestra una trayectoria completa que NUNCA se borra
    """
    
    def __init__(self):
        # Buffers SIN LÍMITE - mantiene TODOS los puntos
        self.x_data = deque()  # ✅ Sin maxlen
        self.y_data = deque()  # ✅ Sin maxlen  
        self.theta_data = deque()  # ✅ Sin maxlen
        self.time_data = deque()  # ✅ Sin maxlen
        
        # Variables actuales
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Control
        self.running = True
        self.data_lock = threading.Lock()
        
        # Límites dinámicos
        self.x_min_ever = float('inf')
        self.x_max_ever = float('-inf')
        self.y_min_ever = float('inf')
        self.y_max_ever = float('-inf')
        self.limits_initialized = False
        
        # Configuración visual
        self.cone_length = 1.5
        self.cone_angle = 0.3
        
        # Estadísticas
        self.start_time = time.time()
    
    def generate_complex_trajectory(self):
        """Genera una trayectoria compleja para demostrar que se mantiene"""
        print("🎯 Generando trayectoria compleja...")
        print("📊 Observa cómo NUNCA se borran los puntos anteriores")
        print("⏰ La demo durará varios minutos con diferentes patrones")
        print()
        
        x, y, theta = 0.0, 0.0, 0.0
        t = 0.0
        pattern = 0
        
        while self.running:
            dt = 0.05  # Más puntos por segundo
            
            # Diferentes patrones de movimiento
            if t < 10.0:  # Línea recta
                if pattern != 1:
                    pattern = 1
                    print(f"🔵 Patrón 1: Línea recta (puntos: {len(self.x_data)})")
                v, w = 2.0, 0.0
                
            elif t < 15.0:  # Círculo
                if pattern != 2:
                    pattern = 2
                    print(f"🟣 Patrón 2: Círculo (puntos: {len(self.x_data)})")
                v, w = 1.5, 1.0
                
            elif t < 25.0:  # Zigzag
                if pattern != 3:
                    pattern = 3
                    print(f"🟡 Patrón 3: Zigzag (puntos: {len(self.x_data)})")
                v = 2.0
                w = 3.0 * math.sin(t * 2.0)
                
            elif t < 35.0:  # Espiral
                if pattern != 4:
                    pattern = 4
                    print(f"🟠 Patrón 4: Espiral (puntos: {len(self.x_data)})")
                v = 1.0 + t * 0.05
                w = 0.8
                
            elif t < 45.0:  # Cuadrado
                if pattern != 5:
                    pattern = 5
                    print(f"🔴 Patrón 5: Cuadrado (puntos: {len(self.x_data)})")
                segment = int((t - 35) / 2.5) % 4
                if segment == 0 or segment == 2:
                    v, w = 2.0, 0.0  # Recto
                else:
                    v, w = 0.0, 1.57  # Giro 90°
                    
            elif t < 60.0:  # Figura de 8
                if pattern != 6:
                    pattern = 6
                    print(f"🟢 Patrón 6: Figura de 8 (puntos: {len(self.x_data)})")
                v = 1.5
                w = 1.2 * math.sin(t * 0.4)
                
            else:  # Movimiento aleatorio
                if pattern != 7:
                    pattern = 7
                    print(f"🎲 Patrón 7: Aleatorio (puntos: {len(self.x_data)})")
                v = 1.0 + 0.5 * math.sin(t * 0.3)
                w = 0.8 * math.cos(t * 0.7)
                
                if t > 90.0:  # Reiniciar después de 90 segundos
                    t = 0.0
                    x, y, theta = 0.0, 0.0, 0.0
                    print("\n🔄 Reiniciando demo (los puntos anteriores se mantienen)\n")
            
            # Actualizar posición
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            theta = ((theta + math.pi) % (2 * math.pi)) - math.pi
            
            self.add_data_point(x, y, theta)
            
            t += dt
            time.sleep(dt)
    
    def add_data_point(self, x, y, theta):
        """Agrega un punto - SIN LÍMITE"""
        current_time = time.time()
        
        with self.data_lock:
            self.current_x = x
            self.current_y = y
            self.current_theta = theta
            
            # ✅ Agregar SIN límite - se mantienen TODOS los puntos
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
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        self.fig.suptitle('🎯 Demo: Trayectoria Completa - NUNCA se borra', fontsize=18, fontweight='bold')
        
        # Gráfica principal
        self.ax1.set_title('🔥 Trayectoria COMPLETA con Ejes Dinámicos', fontweight='bold', fontsize=14)
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.grid(True, alpha=0.3)
        
        # Línea de trayectoria con colores que cambian
        self.trajectory_line, = self.ax1.plot([], [], 'navy', linewidth=2, alpha=0.8, label='Trayectoria Completa')
        self.recent_line, = self.ax1.plot([], [], 'red', linewidth=4, alpha=0.9, label='Últimos 20 puntos')
        self.robot_point, = self.ax1.plot([], [], 'o', color='red', markersize=15, 
                                          markeredgecolor='black', markeredgewidth=2, label='Robot')
        
        # Cono de visión
        self.vision_cone = Polygon([(0, 0)], closed=True, alpha=0.6, 
                                  facecolor='orange', edgecolor='red', linewidth=2)
        self.ax1.add_patch(self.vision_cone)
        
        # Información en tiempo real
        self.stats_text = self.ax1.text(0.02, 0.98, '', transform=self.ax1.transAxes, fontsize=12,
                                       verticalalignment='top', 
                                       bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.9))
        
        self.limits_text = self.ax1.text(0.02, 0.02, '', transform=self.ax1.transAxes, fontsize=11,
                                        verticalalignment='bottom',
                                        bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.9))
        
        self.ax1.legend(loc='upper right')
        
        # Gráficas temporales
        self.ax2.set_title('📈 Posición X vs Tiempo')
        self.ax2.set_xlabel('Tiempo (s)')
        self.ax2.set_ylabel('X (m)')
        self.ax2.grid(True, alpha=0.3)
        self.x_time_line, = self.ax2.plot([], [], 'g-', linewidth=2)
        
        self.ax3.set_title('📈 Posición Y vs Tiempo')
        self.ax3.set_xlabel('Tiempo (s)')
        self.ax3.set_ylabel('Y (m)')
        self.ax3.grid(True, alpha=0.3)
        self.y_time_line, = self.ax3.plot([], [], 'r-', linewidth=2)
        
        self.ax4.set_title('🧭 Orientación Theta vs Tiempo')
        self.ax4.set_xlabel('Tiempo (s)')
        self.ax4.set_ylabel('Theta (rad)')
        self.ax4.grid(True, alpha=0.3)
        self.theta_time_line, = self.ax4.plot([], [], 'm-', linewidth=2)
    
    def update_plots(self, frame):
        """Actualiza las gráficas"""
        with self.data_lock:
            total_points = len(self.x_data)
            
            if total_points > 0:
                # ✅ Mostrar TODA la trayectoria (nunca se borra)
                self.trajectory_line.set_data(list(self.x_data), list(self.y_data))
                
                # Destacar últimos 20 puntos
                recent_count = min(20, total_points)
                if recent_count > 0:
                    recent_x = list(self.x_data)[-recent_count:]
                    recent_y = list(self.y_data)[-recent_count:]
                    self.recent_line.set_data(recent_x, recent_y)
                
                # Robot actual
                self.robot_point.set_data([self.current_x], [self.current_y])
                
                # Cono de visión
                cone_points = self.calculate_cone(self.current_x, self.current_y, self.current_theta)
                self.vision_cone.set_xy(cone_points)
                
                # Ejes dinámicos que se expanden
                if self.limits_initialized:
                    x_range = max(self.x_max_ever - self.x_min_ever, 1.0)
                    y_range = max(self.y_max_ever - self.y_min_ever, 1.0)
                    
                    margin_x = max(x_range * 0.1, 1.0)
                    margin_y = max(y_range * 0.1, 1.0)
                    
                    new_x_min = self.x_min_ever - margin_x
                    new_x_max = self.x_max_ever + margin_x
                    new_y_min = self.y_min_ever - margin_y
                    new_y_max = self.y_max_ever + margin_y
                    
                    # Solo expandir, nunca contraer
                    current_xlim = self.ax1.get_xlim()
                    current_ylim = self.ax1.get_ylim()
                    
                    final_x_min = min(current_xlim[0], new_x_min)
                    final_x_max = max(current_xlim[1], new_x_max)
                    final_y_min = min(current_ylim[0], new_y_min)
                    final_y_max = max(current_ylim[1], new_y_max)
                    
                    self.ax1.set_xlim(final_x_min, final_x_max)
                    self.ax1.set_ylim(final_y_min, final_y_max)
                    
                    # Estadísticas en tiempo real
                    elapsed = time.time() - self.start_time
                    rate = total_points / elapsed if elapsed > 0 else 0
                    
                    stats = f"🎯 TRAYECTORIA COMPLETA\n"
                    stats += f"📍 Posición: ({self.current_x:.1f}, {self.current_y:.1f})\n"
                    stats += f"🧭 Orientación: {math.degrees(self.current_theta):.0f}°\n"
                    stats += f"📊 Total puntos: {total_points}\n"
                    stats += f"⏱️  Tiempo: {elapsed:.1f}s\n"
                    stats += f"📈 Tasa: {rate:.1f} pts/s"
                    self.stats_text.set_text(stats)
                    
                    limits_info = f"📐 Área total explorada:\n"
                    limits_info += f"   X: [{self.x_min_ever:.1f}, {self.x_max_ever:.1f}] ({x_range:.1f}m)\n"
                    limits_info += f"   Y: [{self.y_min_ever:.1f}, {self.y_max_ever:.1f}] ({y_range:.1f}m)\n"
                    limits_info += f"💾 Memoria: ~{total_points * 32 / 1024:.1f} KB"
                    self.limits_text.set_text(limits_info)
                
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
                                data_margin = abs(data_max - data_min) * 0.1 or 0.5
                                
                                ax.set_xlim(time_min, time_max + 1)
                                ax.set_ylim(data_min - data_margin, data_max + data_margin)
        
        return (self.trajectory_line, self.recent_line, self.robot_point, self.vision_cone,
                self.x_time_line, self.y_time_line, self.theta_time_line)
    
    def start_demo(self):
        """Inicia el demo"""
        print("=" * 60)
        print("🎯 DEMO: TRAYECTORIA COMPLETA - NUNCA SE BORRA")
        print("=" * 60)
        print("✅ Los puntos se mantienen para SIEMPRE")
        print("🔥 La trayectoria crece sin límite")
        print("📈 Los ejes se expanden automáticamente")
        print("⏰ Varios patrones de movimiento programados")
        print("🎮 Presiona Ctrl+C o cierra ventana para salir")
        print("=" * 60)
        print()
        
        self.setup_plots()
        
        # Hilo de generación de datos
        data_thread = threading.Thread(target=self.generate_complex_trajectory, daemon=True)
        data_thread.start()
        
        # Animación con alta frecuencia
        ani = animation.FuncAnimation(self.fig, self.update_plots, interval=50, blit=False)
        
        plt.tight_layout()
        plt.show()
    
    def stop(self):
        """Detiene el demo"""
        self.running = False
        plt.close('all')

def main():
    demo = TrayectoriaCompletaDemo()
    
    try:
        demo.start_demo()
    except KeyboardInterrupt:
        print("\n🛑 Demo detenido por el usuario")
    finally:
        demo.stop()

if __name__ == "__main__":
    main()