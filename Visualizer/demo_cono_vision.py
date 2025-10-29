#!/usr/bin/env python3
"""
Demo mejorado del visualizador con cono de visión

Este ejemplo muestra el robot con un cono de visión que indica
claramente su orientación y dirección de movimiento.
"""

import time
import threading
import math
import numpy as np
from odometria_visualizer import OdometriaVisualizer
from simulation_utils import OdometryDataGenerator

class EnhancedMockSerialConnection:
    """
    Simulación mejorada que genera movimientos más interesantes
    para demostrar mejor el cono de visión
    """
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.time = 0.0
        self.is_open = True
        self.last_update = time.time()
        self.movement_phase = 0
        
    def generate_movement_pattern(self, dt=0.1):
        """
        Genera patrones de movimiento interesantes para mostrar el cono
        """
        # Diferentes fases de movimiento (más rápidas y dinámicas)
        if self.movement_phase < 100:
            # Fase 1: Movimiento recto hacia adelante
            v = 3.0
            w = 0.0
        elif self.movement_phase < 150:
            # Fase 2: Giro en el lugar
            v = 0.2
            w = 1.5
        elif self.movement_phase < 250:
            # Fase 3: Movimiento curvo (círculo)
            v = 2.5
            w = 0.8
        elif self.movement_phase < 350:
            # Fase 4: Zigzag rápido
            v = 2.0
            w = 1.2 * math.sin(self.time * 4)
        else:
            # Fase 5: Espiral dinámica
            v = 2.0 + 0.8 * math.sin(self.time * 0.8)
            w = 0.6 + 0.4 * math.cos(self.time * 0.5)
        
        # Actualizar odometría
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        
        # Normalizar theta
        self.theta = ((self.theta + math.pi) % (2 * math.pi)) - math.pi
        
        self.time += dt
        self.movement_phase += 1
        
        # Resetear después de completar el ciclo
        if self.movement_phase > 400:
            self.movement_phase = 0
    
    def readline(self):
        """Simula readline() del puerto serial"""
        current_time = time.time()
        if current_time - self.last_update > 0.05:  # Más rápido (20 FPS)
            self.generate_movement_pattern(dt=0.05)  # Tiempo más pequeño para suavidad
            data_line = f"x:{self.x:.4f},y:{self.y:.4f},theta:{self.theta:.4f}\n"
            self.last_update = current_time
            return data_line.encode('utf-8')
        return b""
    
    @property
    def in_waiting(self):
        """Simula datos disponibles en buffer"""
        return 1
    
    def close(self):
        """Simula cerrar conexión"""
        self.is_open = False

class VisionConeVisualizer(OdometriaVisualizer):
    """
    Versión especializada del visualizador con mejoras para el cono de visión
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Configuraciones específicas para mejor visualización del cono
        self.cone_length = 4.0      # Cono más largo para mejor visibilidad
        self.cone_angle = 0.5       # Ángulo optimizado
        self.arrow_length = 3.0     # Flecha más larga
        self.update_counter = 0     # Contador para optimizar updates
        
    def setup_plots(self):
        """Configura las gráficas con mejoras visuales"""
        super().setup_plots()
        
        # Personalizar la gráfica de trayectoria para mejor visualización
        self.ax1.set_title('Trayectoria con Cono de Visión', fontsize=14, fontweight='bold')
        
        # Cambiar colores para mejor contraste
        self.trajectory_line.set_color('navy')
        self.trajectory_line.set_linewidth(3)
        
        self.current_pos.set_color('darkgreen')
        self.current_pos.set_markersize(12)
        self.current_pos.set_marker('o')
        self.current_pos.set_markeredgecolor('black')
        self.current_pos.set_markeredgewidth(2)
        
        # Mejorar el cono de visión
        self.vision_cone.set_facecolor('lightcoral')
        self.vision_cone.set_alpha(0.6)
        self.vision_cone.set_edgecolor('red')
        self.vision_cone.set_linewidth(2)
        
        # Mejorar la flecha de dirección
        self.direction_arrow.set_color('red')
        self.direction_arrow.set_linewidth(4)
        self.direction_arrow.set_alpha(0.9)
        
        # Añadir texto informativo
        self.info_text = self.ax1.text(0.02, 0.98, '', transform=self.ax1.transAxes,
                                     verticalalignment='top', fontsize=10,
                                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    def calculate_vision_cone(self, x, y, theta, cone_length=None, cone_angle=None):
        """Versión mejorada del cálculo del cono"""
        if cone_length is None:
            cone_length = self.cone_length
        if cone_angle is None:
            cone_angle = self.cone_angle
            
        return super().calculate_vision_cone(x, y, theta, cone_length, cone_angle)
    
    def update_plots(self, frame):
        """Actualización optimizada con información adicional"""
        result = super().update_plots(frame)
        
        # Actualizar texto informativo solo cada 5 frames (optimización)
        self.update_counter += 1
        if self.update_counter % 5 == 0:
            with self.data_lock:
                if len(self.x_data) > 0:
                    info_text = f"Posición: ({self.current_x:.2f}, {self.current_y:.2f})\n"
                    info_text += f"Orientación: {self.current_theta:.2f} rad ({math.degrees(self.current_theta):.1f}°)\n"
                    info_text += f"Puntos: {len(self.x_data)} | Velocidad: {20:.0f} FPS"
                    self.info_text.set_text(info_text)
        
        return result
    
    def connect_serial(self):
        """Conecta con la simulación mejorada"""
        try:
            self.serial_connection = EnhancedMockSerialConnection()
            print(f"Conectado a simulador mejorado con cono de visión")
            return True
        except Exception as e:
            print(f"Error al conectar simulador: {e}")
            return False

def main():
    """Función principal del demo mejorado"""
    print("=== Demo Mejorado: Visualizador con Cono de Visión ===")
    print("Este demo muestra:")
    print("🔴 Cono rojo semitransparente = Campo de visión del robot")
    print("🔴 Línea roja gruesa = Dirección principal")
    print("🟢 Punto verde = Posición actual del robot")
    print("🔵 Línea azul = Trayectoria seguida")
    print()
    print("El robot ejecutará diferentes patrones de movimiento:")
    print("1. Movimiento recto")
    print("2. Giro en el lugar") 
    print("3. Movimiento circular")
    print("4. Movimiento zigzag")
    print("5. Movimiento en espiral")
    print("\nObserva cómo el cono de visión indica la orientación!")
    print("Presiona Ctrl+C para detener\n")
    
    # Crear visualizador mejorado
    visualizer = VisionConeVisualizer(
        port='SIMULADOR_MEJORADO',
        baudrate=9600,
        buffer_size=150  # Buffer optimizado para mejor rendimiento
    )
    
    try:
        visualizer.start_visualization()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        visualizer.stop()

if __name__ == "__main__":
    main()