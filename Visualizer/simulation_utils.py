import serial
import time
import math
import random

def simulate_odometry_data(port='COM4', baudrate=9600):
    """
    Simula datos de odometría para testing sin hardware real.
    Esta función no es necesaria si tienes datos reales llegando por COM4.
    
    Para usar esta simulación, necesitarías un puerto virtual o 
    modificar el código principal para generar datos sintéticos.
    """
    
    # Parámetros de simulación
    t = 0
    x = 0
    y = 0
    theta = 0
    
    # Velocidades simuladas
    v_linear = 0.5  # m/s
    v_angular = 0.1  # rad/s
    
    dt = 0.1  # Intervalo de tiempo
    
    print("Simulando datos de odometría...")
    print("Formato: x:valor,y:valor,theta:valor")
    
    try:
        # En un caso real, esto se conectaría al puerto serial
        # ser = serial.Serial(port, baudrate)
        
        while True:
            # Simular movimiento circular
            v_lin = v_linear + random.uniform(-0.1, 0.1)
            v_ang = v_angular + random.uniform(-0.05, 0.05)
            
            # Actualizar odometría (modelo cinemático diferencial simple)
            x += v_lin * math.cos(theta) * dt
            y += v_lin * math.sin(theta) * dt
            theta += v_ang * dt
            
            # Normalizar theta entre -pi y pi
            theta = ((theta + math.pi) % (2 * math.pi)) - math.pi
            
            # Formato de salida
            data_string = f"x:{x:.3f},y:{y:.3f},theta:{theta:.3f}"
            
            print(data_string)
            
            # En caso real, enviarías por serial:
            # ser.write((data_string + '\n').encode())
            
            time.sleep(dt)
            t += dt
            
    except KeyboardInterrupt:
        print("\nSimulación detenida")
    except Exception as e:
        print(f"Error en simulación: {e}")


class OdometryDataGenerator:
    """
    Generador de datos sintéticos para probar el visualizador
    sin necesidad de hardware real
    """
    
    def __init__(self):
        self.reset()
    
    def reset(self):
        """Reinicia los valores de odometría"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.time = 0.0
    
    def update(self, dt=0.1):
        """
        Actualiza la odometría simulando un movimiento
        
        Args:
            dt (float): Intervalo de tiempo
            
        Returns:
            tuple: (x, y, theta)
        """
        # Simular velocidades variables
        v_base = 1.0
        w_base = 0.3
        
        # Añadir algo de ruido y variación
        v = v_base + 0.3 * math.sin(self.time * 0.5) + random.uniform(-0.1, 0.1)
        w = w_base * math.cos(self.time * 0.3) + random.uniform(-0.05, 0.05)
        
        # Modelo cinemático
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        
        # Normalizar ángulo
        self.theta = ((self.theta + math.pi) % (2 * math.pi)) - math.pi
        
        self.time += dt
        
        return self.x, self.y, self.theta
    
    def get_formatted_string(self):
        """
        Obtiene los datos en el formato esperado por el visualizador
        
        Returns:
            str: Cadena formateada "x:valor,y:valor,theta:valor"
        """
        return f"x:{self.x:.4f},y:{self.y:.4f},theta:{self.theta:.4f}"


def test_data_parsing():
    """Prueba el parseo de diferentes formatos de datos"""
    from odometria_visualizer import OdometriaVisualizer
    
    visualizer = OdometriaVisualizer()
    
    # Casos de prueba
    test_cases = [
        "x:1.234,y:5.678,theta:0.789",
        "X:2.345,Y:6.789,THETA:1.234",
        "x: 3.456 , y: 7.890 , theta: 2.345",
        "1.111,2.222,3.333",
        "x:4.567,y:8.901,theta:3.456\n",
        "invalid format",
        "",
    ]
    
    print("Probando parseo de datos:")
    print("-" * 50)
    
    for i, test_case in enumerate(test_cases):
        result = visualizer.parse_serial_data(test_case)
        print(f"Test {i+1}: '{test_case}' -> {result}")
    
    print("-" * 50)


if __name__ == "__main__":
    print("=== Utilidades para Simulación de Odometría ===")
    print("1. Para probar el parseo de datos:")
    test_data_parsing()
    
    print("\n2. Para generar datos sintéticos:")
    generator = OdometryDataGenerator()
    
    for i in range(10):
        x, y, theta = generator.update()
        formatted = generator.get_formatted_string()
        print(f"Step {i+1}: {formatted}")
    
    print("\n3. Para simulación continua, ejecuta:")
    print("   python simulation_utils.py --simulate")
    
    # Simulación continua si se pasa el argumento
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == '--simulate':
        simulate_odometry_data()