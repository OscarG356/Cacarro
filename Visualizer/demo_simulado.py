#!/usr/bin/env python3
"""
Ejemplo simple de uso del visualizador de odometría

Este archivo demuestra cómo usar el visualizador con datos simulados
cuando no tienes hardware real conectado.
"""

import time
import threading
from odometria_visualizer import OdometriaVisualizer
from simulation_utils import OdometryDataGenerator

class MockSerialConnection:
    """
    Simula una conexión serial para testing sin hardware
    """
    
    def __init__(self, data_generator):
        self.generator = data_generator
        self.is_open = True
        self.buffer = ""
        self.last_update = time.time()
        
    def readline(self):
        """Simula readline() del puerto serial"""
        # Generar nuevos datos cada 100ms
        current_time = time.time()
        if current_time - self.last_update > 0.1:
            self.generator.update()
            data_line = self.generator.get_formatted_string() + "\n"
            self.last_update = current_time
            return data_line.encode('utf-8')
        return b""
    
    @property
    def in_waiting(self):
        """Simula datos disponibles en buffer"""
        return 1  # Siempre hay datos disponibles
    
    def close(self):
        """Simula cerrar conexión"""
        self.is_open = False

class MockOdometriaVisualizer(OdometriaVisualizer):
    """
    Versión modificada del visualizador que usa datos simulados
    """
    
    def connect_serial(self):
        """Conecta con datos simulados en lugar de puerto real"""
        try:
            self.data_generator = OdometryDataGenerator()
            self.serial_connection = MockSerialConnection(self.data_generator)
            print(f"Conectado a simulador (en lugar de {self.port})")
            return True
        except Exception as e:
            print(f"Error al conectar simulador: {e}")
            return False

def main():
    """Función principal de demostración"""
    print("=== Demo del Visualizador de Odometría ===")
    print("Este ejemplo usa datos simulados para demostrar el funcionamiento")
    print("sin necesidad de hardware real conectado.")
    print("\nPresiona Ctrl+C para detener\n")
    
    # Crear visualizador con datos simulados
    visualizer = MockOdometriaVisualizer(
        port='SIMULADO',
        baudrate=9600,
        buffer_size=150
    )
    
    try:
        visualizer.start_visualization()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        visualizer.stop()

if __name__ == "__main__":
    main()