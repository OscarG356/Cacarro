#!/usr/bin/env python3
"""
Demo ultra-optimizado con efectos visuales espectaculares
"""

import time
import threading
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import animation
import serial
from collections import deque
import re

class FastOdometryVisualizer:
    """Visualizador ultra-optimizado para máximo rendimiento"""
    
    def __init__(self, buffer_size=100):
        self.buffer_size = buffer_size
        
        # Buffers optimizados
        self.x_data = deque(maxlen=buffer_size)
        self.y_data = deque(maxlen=buffer_size)
        self.theta_data = deque(maxlen=buffer_size)
        
        # Estado actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Control optimizado
        self.running = True
        self.data_lock = threading.Lock()
        
        # Configuración del cono
        self.cone_length = 5.0
        self.cone_angle = 0.4
        
        # Simulador integrado
        self.setup_simulator()
        
    def setup_simulator(self):
        """Simulador integrado ultra-rápido"""
        self.sim_x = 0.0
        self.sim_y = 0.0 
        self.sim_theta = 0.0
        self.sim_time = 0.0
        self.sim_phase = 0
        
    def generate_data(self):
        """Generador de datos con movimientos más visibles"""
        dt = 0.05  # Tiempo más grande para movimientos más notorios
        
        # Patrones de movimiento MÁS CLAROS y VISIBLES
        if self.sim_phase < 100:
            # Movimiento recto LARGO para ver cambio en X
            v = 2.0  
            w = 0.0
        elif self.sim_phase < 150:
            # Giro completo en el lugar para ver cambio en theta
            v = 0.0  # SIN movimiento lineal
            w = 1.0  # Giro moderado
        elif self.sim_phase < 250:
            # Movimiento en Y (hacia arriba)
            v = 2.0
            w = 0.0
            # Forzar movimiento perpendicular
            if self.sim_phase == 150:  # Al inicio de esta fase
                self.sim_theta = math.pi / 2  # Apuntar hacia arriba (90°)
        elif self.sim_phase < 300:
            # Círculo GRANDE para ver trayectoria curva
            v = 1.5
            w = 0.3  # Giro lento para círculo grande
        elif self.sim_phase < 400:
            # Movimiento diagonal (hacia abajo-derecha)
            v = 2.0
            w = 0.0
            if self.sim_phase == 300:
                self.sim_theta = -math.pi / 4  # -45°
        else:
            # Zigzag AMPLIO
            v = 1.8
            w = 0.8 * math.sin(self.sim_time * 1.5)  # Zigzag más lento y amplio
        
        # Actualizar simulación
        self.sim_x += v * math.cos(self.sim_theta) * dt
        self.sim_y += v * math.sin(self.sim_theta) * dt
        self.sim_theta += w * dt
        self.sim_theta = ((self.sim_theta + math.pi) % (2 * math.pi)) - math.pi
        
        self.sim_time += dt
        self.sim_phase += 1
        
        # Reset cíclico
        if self.sim_phase > 500:
            self.sim_phase = 0
            # Reiniciar posición para nuevo ciclo
            self.sim_x = 0.0
            self.sim_y = 0.0
            self.sim_theta = 0.0
            
        return self.sim_x, self.sim_y, self.sim_theta
    
    def setup_plots(self):
        """Setup ultra-optimizado"""
        plt.style.use('dark_background')  # Tema oscuro para mejor visual
        
        self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 10))
        self.fig.patch.set_facecolor('black')
        
        # Configurar eje principal
        self.ax.set_facecolor('black')
        self.ax.set_title('*** ROBOT TRACKING - VISTA EN TIEMPO REAL ***', 
                         fontsize=16, fontweight='bold', color='cyan')
        self.ax.set_xlabel('X (metros)', fontsize=12, color='white')
        self.ax.set_ylabel('Y (metros)', fontsize=12, color='white')
        self.ax.grid(True, alpha=0.3, color='gray')
        self.ax.set_aspect('equal')
        
        # Elementos visuales espectaculares
        self.trail_line, = self.ax.plot([], [], 'cyan', linewidth=3, alpha=0.8, label='Trayectoria')
        self.robot_dot, = self.ax.plot([], [], 'o', markersize=15, color='lime', 
                                      markeredgecolor='white', markeredgewidth=3, label='Robot')
        
        # Cono de visión espectacular
        self.vision_cone = Polygon([(0, 0)], closed=True, alpha=0.6, 
                                  facecolor='red', edgecolor='yellow', linewidth=3)
        self.ax.add_patch(self.vision_cone)
        
        # Vector de dirección
        self.direction_arrow, = self.ax.plot([], [], 'yellow', linewidth=5, alpha=0.9)
        
        # Información en tiempo real
        self.info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                     fontsize=12, color='white', fontweight='bold',
                                     verticalalignment='top',
                                     bbox=dict(boxstyle='round,pad=0.5', facecolor='black', alpha=0.7))
        
        # Velocímetro
        self.speed_text = self.ax.text(0.02, 0.02, '', transform=self.ax.transAxes,
                                      fontsize=14, color='lime', fontweight='bold',
                                      bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.8))
        
        self.ax.legend(loc='upper right', fancybox=True, shadow=True)
        
    def calculate_cone(self, x, y, theta):
        """Cálculo optimizado del cono"""
        # Puntos del cono
        tip_x = x + self.cone_length * np.cos(theta)
        tip_y = y + self.cone_length * np.sin(theta)
        
        left_x = x + self.cone_length * np.cos(theta + self.cone_angle)
        left_y = y + self.cone_length * np.sin(theta + self.cone_angle)
        
        right_x = x + self.cone_length * np.cos(theta - self.cone_angle)
        right_y = y + self.cone_length * np.sin(theta - self.cone_angle)
        
        return np.array([[x, y], [left_x, left_y], [tip_x, tip_y], [right_x, right_y]])
    
    def update_plots(self, frame):
        """Update ultra-optimizado"""
        # Generar nuevos datos
        x, y, theta = self.generate_data()
        
        # Actualizar buffers
        self.current_x, self.current_y, self.current_theta = x, y, theta
        self.x_data.append(x)
        self.y_data.append(y)
        self.theta_data.append(theta)
        
        # Actualizar visualización
        if len(self.x_data) > 0:
            # Trayectoria
            self.trail_line.set_data(list(self.x_data), list(self.y_data))
            
            # Robot
            self.robot_dot.set_data([x], [y])
            
            # Cono de visión
            cone_points = self.calculate_cone(x, y, theta)
            self.vision_cone.set_xy(cone_points)
            
            # Flecha direccional
            arrow_end_x = x + 4.0 * np.cos(theta)
            arrow_end_y = y + 4.0 * np.sin(theta)
            self.direction_arrow.set_data([x, arrow_end_x], [y, arrow_end_y])
            
            # Vista adaptativa: muestra la trayectoria completa + área alrededor del robot
            if len(self.x_data) > 10:  # Solo después de tener suficientes puntos
                # Calcular límites de la trayectoria
                x_min, x_max = min(self.x_data), max(self.x_data)
                y_min, y_max = min(self.y_data), max(self.y_data)
                
                # Calcular el tamaño necesario para mostrar toda la trayectoria
                x_range = x_max - x_min
                y_range = y_max - y_min
                
                # Agregar margen y asegurar que el robot esté visible
                margin = max(x_range, y_range, 10.0) * 0.3  # Al menos 10m de margen
                
                # Expandir límites para incluir robot + trayectoria
                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2
                
                # Asegurar que el robot actual esté en la vista
                x_center = (x_center + x) / 2  # Promedio entre centro de trayectoria y robot
                y_center = (y_center + y) / 2
                
                total_range = max(x_range + margin, y_range + margin, 15.0)
                
                self.ax.set_xlim(x_center - total_range/2, x_center + total_range/2)
                self.ax.set_ylim(y_center - total_range/2, y_center + total_range/2)
            else:
                # Vista inicial centrada en el robot
                window_size = 15.0
                self.ax.set_xlim(x - window_size/2, x + window_size/2)
                self.ax.set_ylim(y - window_size/2, y + window_size/2)
            
            # Información en tiempo real (actualizar solo cada 3 frames)
            if frame % 3 == 0:
                # Calcular velocidad aproximada
                if len(self.x_data) > 1:
                    dx = self.x_data[-1] - self.x_data[-2] if len(self.x_data) > 1 else 0
                    dy = self.y_data[-1] - self.y_data[-2] if len(self.y_data) > 1 else 0
                    speed = math.sqrt(dx*dx + dy*dy) / 0.03  # velocidad en m/s
                else:
                    speed = 0
                
                info = f"Posicion: ({x:.1f}, {y:.1f})\n"
                info += f"Orientacion: {math.degrees(theta):.0f}°\n" 
                info += f"Puntos: {len(self.x_data)}"
                self.info_text.set_text(info)
                
                self.speed_text.set_text(f"Velocidad: {speed:.1f} m/s")
        
        return (self.trail_line, self.robot_dot, self.vision_cone, 
               self.direction_arrow, self.info_text, self.speed_text)
    
    def run(self):
        """Ejecutar visualización ultra-rápida"""
        print("*** INICIANDO DEMO ULTRA-RAPIDO CORREGIDO ***")
        print("Caracteristicas:")
        print("   >> Rendering a 50 FPS")
        print("   >> Vista adaptativa que muestra TODA la trayectoria") 
        print("   >> Movimientos CLAROS y VISIBLES")
        print("   >> Cambios notorios en X, Y y orientacion")
        print("\nFases del movimiento:")
        print("   1. Movimiento recto largo (X cambia)")
        print("   2. Giro en el lugar (theta cambia)")
        print("   3. Movimiento vertical (Y cambia)")  
        print("   4. Circulo grande (X,Y,theta cambian)")
        print("   5. Movimiento diagonal")
        print("   6. Zigzag amplio")
        print("\nControles: Ctrl+C para salir\n")
        
        self.setup_plots()
        
        # Animación ultra-rápida
        ani = animation.FuncAnimation(
            self.fig, self.update_plots, 
            interval=20,  # 50 FPS
            blit=True,
            cache_frame_data=False
        )
        
        plt.tight_layout()
        plt.show()

def main():
    visualizer = FastOdometryVisualizer(buffer_size=200)  # Buffer más grande para ver más trayectoria
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\n*** Demo detenido. Gracias! ***")
    except Exception as e:
        print(f"*** Error: {e} ***")

if __name__ == "__main__":
    main()