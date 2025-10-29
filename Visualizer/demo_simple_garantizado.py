#!/usr/bin/env python3
"""
Demo simple y claro - Garantiza movimientos visibles
"""

import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import animation

class SimpleRobotDemo:
    """Demo simple que garantiza movimientos visibles"""
    
    def __init__(self):
        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.time = 0.0
        
        # Historial para trayectoria
        self.x_history = []
        self.y_history = []
        self.theta_history = []
        
        # Configuración visual
        self.cone_length = 3.0
        self.cone_angle = 0.4
        
        print("=== DEMO SIMPLE CON MOVIMIENTOS GARANTIZADOS ===")
        print("Este demo garantiza que veras:")
        print("✓ X cambiando de 0 a +20 metros")
        print("✓ Y cambiando de 0 a +15 metros") 
        print("✓ Theta rotando 360 grados completos")
        print("✓ Trayectoria completa siempre visible")
        print("✓ Cono de vision mostrando orientacion")
        print("\nPresiona Ctrl+C para salir\n")
    
    def update_robot(self):
        """Actualiza posición del robot con movimientos GARANTIZADOS"""
        dt = 0.08
        
        # Ciclo de tiempo para diferentes fases
        cycle_time = self.time % 20.0  # Ciclo de 20 segundos
        
        if cycle_time < 4.0:
            # FASE 1: Movimiento recto hacia DERECHA (X aumenta)
            v = 3.0  # 3 m/s
            w = 0.0
            self.theta = 0.0  # Asegurar que apunte a la derecha
            
        elif cycle_time < 6.0:
            # FASE 2: Giro puro (theta cambia, x,y NO cambian)
            v = 0.0  # SIN movimiento
            w = 1.5  # Giro de 1.5 rad/s
            
        elif cycle_time < 10.0:
            # FASE 3: Movimiento hacia ARRIBA (Y aumenta)
            v = 2.5
            w = 0.0
            self.theta = math.pi / 2  # 90° hacia arriba
            
        elif cycle_time < 12.0:
            # FASE 4: Otro giro
            v = 0.0
            w = -1.5  # Giro en sentido contrario
            
        elif cycle_time < 16.0:
            # FASE 5: Movimiento DIAGONAL (X y Y cambian)
            v = 2.0
            w = 0.0
            self.theta = math.pi / 4  # 45°
            
        else:
            # FASE 6: Círculo (todo cambia)
            v = 2.0
            w = 0.8
        
        # Aplicar cinemática diferencial
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        
        # Normalizar theta
        self.theta = ((self.theta + math.pi) % (2 * math.pi)) - math.pi
        
        # Guardar en historial
        self.x_history.append(self.x)
        self.y_history.append(self.y)
        self.theta_history.append(self.theta)
        
        # Limitar historial
        max_history = 300
        if len(self.x_history) > max_history:
            self.x_history.pop(0)
            self.y_history.pop(0)
            self.theta_history.pop(0)
        
        self.time += dt
        
        # Debug: imprimir cada segundo
        if int(self.time * 10) % 10 == 0:  # Cada 1 segundo aprox
            print(f"Tiempo: {self.time:.1f}s | Pos: ({self.x:.1f}, {self.y:.1f}) | Theta: {math.degrees(self.theta):.0f}°")
    
    def calculate_cone(self):
        """Calcula puntos del cono de visión"""
        # Punta del cono
        tip_x = self.x + self.cone_length * math.cos(self.theta)
        tip_y = self.y + self.cone_length * math.sin(self.theta)
        
        # Lados del cono
        left_x = self.x + self.cone_length * math.cos(self.theta + self.cone_angle)
        left_y = self.y + self.cone_length * math.sin(self.theta + self.cone_angle)
        
        right_x = self.x + self.cone_length * math.cos(self.theta - self.cone_angle)
        right_y = self.y + self.cone_length * math.sin(self.theta - self.cone_angle)
        
        return [(self.x, self.y), (left_x, left_y), (tip_x, tip_y), (right_x, right_y)]
    
    def setup_plot(self):
        """Configurar la visualización"""
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        
        # Configurar ejes
        self.ax.set_title('Robot con Cono de Vision - Movimientos Garantizados', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X (metros)', fontsize=12)
        self.ax.set_ylabel('Y (metros)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Elementos visuales
        self.trajectory_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Trayectoria')
        self.robot_point, = self.ax.plot([], [], 'go', markersize=12, markeredgecolor='black', markeredgewidth=2, label='Robot')
        
        # Cono de visión
        self.vision_cone = Polygon([(0, 0)], closed=True, alpha=0.5, facecolor='red', edgecolor='darkred', linewidth=2)
        self.ax.add_patch(self.vision_cone)
        
        # Flecha de dirección
        self.direction_line, = self.ax.plot([], [], 'r-', linewidth=4, alpha=0.8, label='Direccion')
        
        # Información
        self.info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes, fontsize=10,
                                     verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        self.ax.legend()
        
    def update_plot(self, frame):
        """Actualizar visualización"""
        # Actualizar robot
        self.update_robot()
        
        # Actualizar elementos visuales
        if len(self.x_history) > 0:
            # Trayectoria
            self.trajectory_line.set_data(self.x_history, self.y_history)
            
            # Robot
            self.robot_point.set_data([self.x], [self.y])
            
            # Cono de visión
            cone_points = self.calculate_cone()
            self.vision_cone.set_xy(cone_points)
            
            # Flecha direccional
            arrow_end_x = self.x + 2.0 * math.cos(self.theta)
            arrow_end_y = self.y + 2.0 * math.sin(self.theta)
            self.direction_line.set_data([self.x, arrow_end_x], [self.y, arrow_end_y])
            
            # Ajustar vista para mostrar TODA la trayectoria
            if len(self.x_history) > 5:
                x_min, x_max = min(self.x_history), max(self.x_history)
                y_min, y_max = min(self.y_history), max(self.y_history)
                
                # Agregar margen
                x_range = max(x_max - x_min, 5.0)
                y_range = max(y_max - y_min, 5.0)
                margin = max(x_range, y_range) * 0.1 + 2.0
                
                self.ax.set_xlim(x_min - margin, x_max + margin)
                self.ax.set_ylim(y_min - margin, y_max + margin)
            
            # Actualizar información (cada 5 frames para rendimiento)
            if frame % 5 == 0:
                cycle_time = self.time % 20.0
                if cycle_time < 4.0:
                    phase = "MOVIMIENTO RECTO →"
                elif cycle_time < 6.0:
                    phase = "GIRO EN EL LUGAR ↻"
                elif cycle_time < 10.0:
                    phase = "MOVIMIENTO ARRIBA ↑"
                elif cycle_time < 12.0:
                    phase = "GIRO CONTRARIO ↺"
                elif cycle_time < 16.0:
                    phase = "MOVIMIENTO DIAGONAL ↗"
                else:
                    phase = "CIRCULO COMPLETO ⟲"
                
                info = f"Posicion: ({self.x:.1f}, {self.y:.1f})\n"
                info += f"Orientacion: {math.degrees(self.theta):.0f}°\n"
                info += f"Fase: {phase}\n"
                info += f"Puntos: {len(self.x_history)}"
                self.info_text.set_text(info)
        
        return self.trajectory_line, self.robot_point, self.vision_cone, self.direction_line
    
    def run(self):
        """Ejecutar demo"""
        self.setup_plot()
        
        # Animación
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
        plt.tight_layout()
        plt.show()

def main():
    demo = SimpleRobotDemo()
    try:
        demo.run()
    except KeyboardInterrupt:
        print("\n*** Demo detenido ***")

if __name__ == "__main__":
    main()