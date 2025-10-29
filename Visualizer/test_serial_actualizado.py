#!/usr/bin/env python3
"""
Test del visualizador actualizado con datos simulados
para verificar que funciona con ejes dinámicos
"""

from odometria_visualizer import OdometriaVisualizer
import time
import math

class TestSerialConnection:
    """Simula conexión serial para probar el visualizador actualizado"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.time = 0.0
        self.is_open = True
        self.last_update = time.time()
        
    def readline(self):
        current_time = time.time()
        if current_time - self.last_update > 0.1:  # 10 Hz
            # Generar movimiento de prueba
            dt = 0.1
            
            if self.time < 5.0:
                # Movimiento recto
                v, w = 2.0, 0.0
            elif self.time < 8.0:
                # Giro
                v, w = 0.0, 1.0
            elif self.time < 13.0:
                # Movimiento diagonal
                v, w = 1.5, 0.0
                self.theta = math.pi / 4
            else:
                # Círculo
                v, w = 1.0, 0.5
            
            # Actualizar posición
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            self.theta += w * dt
            self.theta = ((self.theta + math.pi) % (2 * math.pi)) - math.pi
            
            self.time += dt
            self.last_update = current_time
            
            # Formato de datos como COM4
            data_line = f"x:{self.x:.4f},y:{self.y:.4f},theta:{self.theta:.4f}\n"
            return data_line.encode('utf-8')
        
        return b""
    
    @property
    def in_waiting(self):
        return 1
    
    def close(self):
        self.is_open = False

class TestOdometriaVisualizer(OdometriaVisualizer):
    """Versión de prueba del visualizador"""
    
    def connect_serial(self):
        """Conecta con datos de prueba"""
        try:
            self.serial_connection = TestSerialConnection()
            print(f"✓ Conectado a simulador de prueba (en lugar de {self.port})")
            print("  Probando ejes dinámicos con movimientos reales...")
            return True
        except Exception as e:
            print(f"✗ Error al conectar: {e}")
            return False

def main():
    print("=== PRUEBA DEL VISUALIZADOR ACTUALIZADO ===")
    print("Verificando que el código serial tiene ejes dinámicos...")
    print()
    
    visualizer = TestOdometriaVisualizer(
        port='TEST',
        baudrate=9600,
        buffer_size=200
    )
    
    try:
        visualizer.start_visualization()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        visualizer.stop()

if __name__ == "__main__":
    main()