#!/usr/bin/env python3
"""
Demo con ejes dinámicos que SIEMPRE muestran toda la trayectoria
Los ejes se expanden automáticamente pero NUNCA se contraen
"""

import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib import animation

class DynamicAxisRobotDemo:
    """Demo con ejes que crecen dinámicamente y nunca se contraen"""
    
    def __init__(self):
        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.time = 0.0
        
        # Historial completo (sin límite)
        self.x_history = []
        self.y_history = []
        self.theta_history = []
        
        # Límites de ejes (se expanden pero NUNCA se contraen)
        self.x_min_ever = 0.0
        self.x_max_ever = 0.0
        self.y_min_ever = 0.0
        self.y_max_ever = 0.0
        
        # Configuración visual
        self.cone_length = 2.0
        self.cone_angle = 0.3
        
        print("=== DEMO CON EJES DINAMICOS ===")
        print("Características:")
        print("✓ Los ejes se expanden AUTOMATICAMENTE")
        print("✓ Los ejes NUNCA se contraen")
        print("✓ SIEMPRE puedes ver toda la trayectoria")
        print("✓ La vista crece dinámicamente")
        print("✓ Movimientos garantizados en X, Y y theta")
        print("\nPresiona Ctrl+C para salir\n")
    
    def update_robot(self):
        """Actualiza posición con movimientos claros"""
        dt = 0.1  # Tiempo más lento para movimientos más claros
        
        # Ciclo de tiempo para diferentes fases (más largo)
        cycle_time = self.time % 30.0  # Ciclo de 30 segundos
        
        if cycle_time < 6.0:
            # FASE 1: Movimiento recto LARGO hacia derecha
            v = 2.0
            w = 0.0
            self.theta = 0.0  # Forzar dirección este
            
        elif cycle_time < 9.0:
            # FASE 2: Giro completo en el lugar
            v = 0.0  # SIN movimiento
            w = 1.0  # Giro
            
        elif cycle_time < 15.0:
            # FASE 3: Movimiento hacia ARRIBA
            v = 1.8
            w = 0.0
            self.theta = math.pi / 2  # Forzar dirección norte
            
        elif cycle_time < 18.0:
            # FASE 4: Giro 180°
            v = 0.0
            w = -1.2
            
        elif cycle_time < 24.0:
            # FASE 5: Movimiento hacia ABAJO (Y negativo)
            v = 2.0
            w = 0.0
            self.theta = -math.pi / 2  # Forzar dirección sur
            
        else:
            # FASE 6: Círculo grande para crear trayectoria curva
            v = 1.5
            w = 0.4
        
        # Aplicar movimiento
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        
        # Normalizar theta
        self.theta = ((self.theta + math.pi) % (2 * math.pi)) - math.pi
        
        # Guardar en historial (SIN LÍMITE)
        self.x_history.append(self.x)
        self.y_history.append(self.y)
        self.theta_history.append(self.theta)
        
        # Actualizar límites MÁXIMOS (solo se expanden, nunca se contraen)
        self.x_min_ever = min(self.x_min_ever, self.x)
        self.x_max_ever = max(self.x_max_ever, self.x)
        self.y_min_ever = min(self.y_min_ever, self.y)
        self.y_max_ever = max(self.y_max_ever, self.y)
        
        self.time += dt
        
        # Debug cada 2 segundos
        if int(self.time * 5) % 10 == 0:
            print(f"T: {self.time:.1f}s | Pos: ({self.x:.1f}, {self.y:.1f}) | "
                  f"Limites X: [{self.x_min_ever:.1f}, {self.x_max_ever:.1f}] | "
                  f"Limites Y: [{self.y_min_ever:.1f}, {self.y_max_ever:.1f}]")
    
    def calculate_cone(self):
        """Calcula puntos del cono de visión"""
        tip_x = self.x + self.cone_length * math.cos(self.theta)
        tip_y = self.y + self.cone_length * math.sin(self.theta)
        
        left_x = self.x + self.cone_length * math.cos(self.theta + self.cone_angle)
        left_y = self.y + self.cone_length * math.sin(self.theta + self.cone_angle)
        
        right_x = self.x + self.cone_length * math.cos(self.theta - self.cone_angle)
        right_y = self.y + self.cone_length * math.sin(self.theta - self.cone_angle)
        
        return [(self.x, self.y), (left_x, left_y), (tip_x, tip_y), (right_x, right_y)]
    
    def setup_plot(self):
        """Configurar visualización"""
        plt.style.use('default')  # Estilo claro
        
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        
        # Configurar ejes
        self.ax.set_title('*** Robot con Ejes Dinámicos - Vista Completa de Trayectoria ***', 
                         fontsize=16, fontweight='bold', color='darkblue')
        self.ax.set_xlabel('X (metros)', fontsize=12)
        self.ax.set_ylabel('Y (metros)', fontsize=12)
        self.ax.grid(True, alpha=0.4, linestyle='--')
        self.ax.set_aspect('equal')
        
        # Elementos visuales
        self.trajectory_line, = self.ax.plot([], [], 'navy', linewidth=3, alpha=0.8, label='Trayectoria Completa')
        self.robot_point, = self.ax.plot([], [], 'o', color='red', markersize=14, 
                                        markeredgecolor='black', markeredgewidth=2, label='Robot Actual')
        
        # Cono de visión más visible
        self.vision_cone = Polygon([(0, 0)], closed=True, alpha=0.6, 
                                  facecolor='orange', edgecolor='red', linewidth=2)
        self.ax.add_patch(self.vision_cone)
        
        # Flecha de dirección
        self.direction_line, = self.ax.plot([], [], 'red', linewidth=4, alpha=0.9, label='Dirección')
        
        # Información detallada
        self.info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes, fontsize=11,
                                     verticalalignment='top', fontweight='bold',
                                     bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8))
        
        # Información de límites
        self.limits_text = self.ax.text(0.02, 0.02, '', transform=self.ax.transAxes, fontsize=10,
                                       verticalalignment='bottom', 
                                       bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow', alpha=0.8))
        
        self.ax.legend(loc='upper right')
        
        # Establecer límites iniciales mínimos
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
    
    def update_plot(self, frame):
        """Actualizar visualización con ejes dinámicos"""
        # Actualizar robot
        self.update_robot()
        
        # Actualizar elementos visuales
        if len(self.x_history) > 0:
            # Trayectoria completa
            self.trajectory_line.set_data(self.x_history, self.y_history)
            
            # Robot actual
            self.robot_point.set_data([self.x], [self.y])
            
            # Cono de visión
            cone_points = self.calculate_cone()
            self.vision_cone.set_xy(cone_points)
            
            # Flecha direccional
            arrow_length = 1.5
            arrow_end_x = self.x + arrow_length * math.cos(self.theta)
            arrow_end_y = self.y + arrow_length * math.sin(self.theta)
            self.direction_line.set_data([self.x, arrow_end_x], [self.y, arrow_end_y])
            
            # *** CLAVE: EJES DINÁMICOS QUE SOLO SE EXPANDEN ***
            # Calcular rangos actuales (inicializar siempre)
            x_range = max(self.x_max_ever - self.x_min_ever, 0.1)
            y_range = max(self.y_max_ever - self.y_min_ever, 0.1)
            
            if len(self.x_history) > 1:
                # Calcular margen (10% del rango total + mínimo 1 metro)
                margin_x = max(x_range * 0.1, 1.0)
                margin_y = max(y_range * 0.1, 1.0)
                
                # Nuevos límites (SIEMPRE incluyen toda la trayectoria histórica)
                new_x_min = self.x_min_ever - margin_x
                new_x_max = self.x_max_ever + margin_x
                new_y_min = self.y_min_ever - margin_y
                new_y_max = self.y_max_ever + margin_y
                
                # Obtener límites actuales
                current_xlim = self.ax.get_xlim()
                current_ylim = self.ax.get_ylim()
                
                # Solo expandir, NUNCA contraer
                final_x_min = min(current_xlim[0], new_x_min)
                final_x_max = max(current_xlim[1], new_x_max)
                final_y_min = min(current_ylim[0], new_y_min)
                final_y_max = max(current_ylim[1], new_y_max)
                
                # Aplicar nuevos límites
                self.ax.set_xlim(final_x_min, final_x_max)
                self.ax.set_ylim(final_y_min, final_y_max)
            
            # Actualizar información (cada 3 frames)
            if frame % 3 == 0:
                cycle_time = self.time % 30.0
                if cycle_time < 6.0:
                    phase = "MOVIMIENTO ESTE (X+)"
                elif cycle_time < 9.0:
                    phase = "GIRO EN LUGAR"
                elif cycle_time < 15.0:
                    phase = "MOVIMIENTO NORTE (Y+)"
                elif cycle_time < 18.0:
                    phase = "GIRO 180°"
                elif cycle_time < 24.0:
                    phase = "MOVIMIENTO SUR (Y-)"
                else:
                    phase = "CIRCULO AMPLIO"
                
                info = f"Posición: ({self.x:.1f}, {self.y:.1f})\n"
                info += f"Orientación: {math.degrees(self.theta):.0f}°\n"
                info += f"Fase: {phase}\n"
                info += f"Puntos trayectoria: {len(self.x_history)}"
                self.info_text.set_text(info)
                
                # Información de límites
                limits_info = f"Rango X: {self.x_min_ever:.1f} a {self.x_max_ever:.1f} ({x_range:.1f}m)\n"
                limits_info += f"Rango Y: {self.y_min_ever:.1f} a {self.y_max_ever:.1f} ({y_range:.1f}m)\n"
                limits_info += f"Vista: Siempre toda la trayectoria"
                self.limits_text.set_text(limits_info)
        
        return self.trajectory_line, self.robot_point, self.vision_cone, self.direction_line
    
    def run(self):
        """Ejecutar demo"""
        self.setup_plot()
        
        # Animación con velocidad moderada
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=80, blit=False)
        
        plt.tight_layout()
        plt.show()

def main():
    demo = DynamicAxisRobotDemo()
    try:
        demo.run()
    except KeyboardInterrupt:
        print("\n*** Demo detenido - Ejes dinámicos funcionando correctamente ***")

if __name__ == "__main__":
    main()