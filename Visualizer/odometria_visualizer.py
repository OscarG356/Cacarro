import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import time
import threading
import re

class OdometriaVisualizer:
    def __init__(self, port='COM6', baudrate=9600, buffer_size=None):
        """
        Inicializa el visualizador de odometría
        
        Args:
            port (str): Puerto serial (por defecto COM6)
            baudrate (int): Velocidad de comunicación serial
            buffer_size (int): Ignorado - ahora mantiene TODA la trayectoria
        """
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        
        # Buffers para almacenar datos - SIN LÍMITE para mantener trayectoria completa
        self.x_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        self.y_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        self.theta_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        self.time_data = deque()  # Sin maxlen - mantiene TODOS los puntos
        
        # Límites dinámicos para ejes que nunca se contraen
        self.x_min_ever = float('inf')
        self.x_max_ever = float('-inf')
        self.y_min_ever = float('inf')
        self.y_max_ever = float('-inf')
        self.limits_initialized = False
        
        # Variables para los datos actuales
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Control de threads
        self.running = True
        self.paused = False
        self.serial_connection = None
        self.data_lock = threading.Lock()
        
        # Configurar matplotlib para modo interactivo
        plt.ion()
    
    def clear_data(self, event=None):
        """Limpia todos los datos y reinicia la visualización"""
        with self.data_lock:
            # Limpiar todos los buffers de datos
            self.x_data.clear()
            self.y_data.clear()
            self.theta_data.clear()
            self.time_data.clear()
            
            # Reiniciar variables de posición actual
            self.current_x = 0.0
            self.current_y = 0.0
            self.current_theta = 0.0
            
            # Reiniciar límites dinámicos
            self.x_min_ever = float('inf')
            self.x_max_ever = float('-inf')
            self.y_min_ever = float('inf')
            self.y_max_ever = float('-inf')
            self.limits_initialized = False
            
            # Limpiar las líneas gráficas
            self.trajectory_line.set_data([], [])
            self.current_pos.set_data([], [])
            self.direction_arrow.set_data([], [])
            self.x_time_line.set_data([], [])
            self.y_time_line.set_data([], [])
            self.theta_time_line.set_data([], [])
            
            # Reiniciar cono de visión
            self.vision_cone.set_xy([(0, 0)])
            
            # Reiniciar ejes a vista inicial
            self.ax1.set_xlim(-2, 2)
            self.ax1.set_ylim(-2, 2)
            
            # Limpiar información de límites
            self.limits_info.set_text("")
            
            # Redibujar
            self.fig.canvas.draw()
            
        print("🧹 Datos limpiados - Visualización reiniciada")
    
    def toggle_pause(self, event=None):
        """Pausa o reanuda la visualización"""
        self.paused = not self.paused
        
        if self.paused:
            self.pause_button.label.set_text('▶️')
            self.pause_button.label.set_fontsize(16)  # Mantener tamaño
            print("⏸️ Visualización pausada")
        else:
            self.pause_button.label.set_text('⏸️')
            self.pause_button.label.set_fontsize(16)  # Mantener tamaño
            print("▶️ Visualización reanudada")
        
        # Redibujar el botón
        self.fig.canvas.draw_idle()
        
    def connect_serial(self):
        """Establece conexión con el puerto serial"""
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            print(f"Conectado exitosamente al puerto {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error al conectar con {self.port}: {e}")
            return False
    
    def parse_serial_data(self, data_string):
        """
        Parsea los datos recibidos por serial.
        Espera formato: "x:valor,y:valor,theta:valor" o similar
        
        Args:
            data_string (str): Cadena de datos recibida por serial
            
        Returns:
            tuple: (x, y, theta) o None si no se puede parsear
        """
        try:
            # Limpiar la cadena
            data_string = data_string.strip()
            
            # Buscar patrones como x:1.23,y:4.56,theta:7.89 (con espacios opcionales)
            pattern = r'x\s*:\s*([+-]?\d*\.?\d+)\s*,\s*y\s*:\s*([+-]?\d*\.?\d+)\s*,\s*theta\s*:\s*([+-]?\d*\.?\d+)'
            match = re.search(pattern, data_string, re.IGNORECASE)
            
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                theta = float(match.group(3))
                return (x, y, theta)
            
            # Intentar formato alternativo separado por comas
            parts = data_string.split(',')
            if len(parts) >= 3:
                x = float(parts[0].strip())
                y = float(parts[1].strip())
                theta = float(parts[2].strip())
                return (x, y, theta)
                
        except (ValueError, IndexError) as e:
            print(f"Error parseando datos: {data_string} - {e}")
        
        return None
    
    def read_serial_data(self):
        """Thread para leer datos del puerto serial continuamente"""
        if not self.connect_serial():
            return
        
        print("Iniciando lectura de datos serial...")
        print("Formato esperado: 'x:valor,y:valor,theta:valor'")
        
        while self.running:
            try:
                if self.serial_connection.in_waiting > 0:
                    # Leer línea del puerto serial
                    line = self.serial_connection.readline().decode('utf-8', errors='ignore')
                    
                    # Parsear los datos
                    parsed_data = self.parse_serial_data(line)
                    
                    if parsed_data:
                        x, y, theta = parsed_data
                        current_time = time.time()
                        
                        # Solo actualizar datos si no está pausado
                        if not self.paused:
                            # Actualizar datos de forma thread-safe
                            with self.data_lock:
                                self.current_x = x
                                self.current_y = y
                                self.current_theta = theta
                                
                                self.x_data.append(x)
                                self.y_data.append(y)
                                self.theta_data.append(theta)
                                self.time_data.append(current_time)
                                
                                # Actualizar límites dinámicos (solo se expanden, nunca se contraen)
                                if not self.limits_initialized:
                                    self.x_min_ever = self.x_max_ever = x
                                    self.y_min_ever = self.y_max_ever = y
                                    self.limits_initialized = True
                                else:
                                    self.x_min_ever = min(self.x_min_ever, x)
                                    self.x_max_ever = max(self.x_max_ever, x)
                                    self.y_min_ever = min(self.y_min_ever, y)
                                    self.y_max_ever = max(self.y_max_ever, y)
                        
                        print(f"Datos: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
                
                time.sleep(0.01)  # Pequeño delay para no sobrecargar el CPU
                
            except serial.SerialException as e:
                print(f"Error de comunicación serial: {e}")
                break
            except Exception as e:
                print(f"Error inesperado: {e}")
                break
    
    def calculate_vision_cone(self, x, y, theta, cone_length=1.0, cone_angle=0.5):
        """
        Calcula los puntos del cono de visión
        
        Args:
            x (float): Posición X del robot
            y (float): Posición Y del robot  
            theta (float): Orientación del robot en radianes
            cone_length (float): Longitud del cono
            cone_angle (float): Ángulo de apertura del cono (radianes)
            
        Returns:
            numpy.array: Puntos del cono como array [[x1,y1], [x2,y2], ...]
        """
        # Punto base del robot
        base_x, base_y = x, y
        
        # Calcular los puntos del cono
        # Dirección principal
        tip_x = base_x + cone_length * np.cos(theta)
        tip_y = base_y + cone_length * np.sin(theta)
        
        # Bordes del cono
        left_angle = theta + cone_angle
        right_angle = theta - cone_angle
        
        left_x = base_x + cone_length * np.cos(left_angle)
        left_y = base_y + cone_length * np.sin(left_angle)
        
        right_x = base_x + cone_length * np.cos(right_angle)
        right_y = base_y + cone_length * np.sin(right_angle)
        
        # Puntos del polígono (base -> izquierda -> punta -> derecha -> base)
        cone_points = np.array([
            [base_x, base_y],
            [left_x, left_y],
            [tip_x, tip_y],
            [right_x, right_y],
            [base_x, base_y]
        ])
        
        return cone_points
    
    def setup_plots(self):
        """Configura las gráficas"""
        # Crear figura con subplots
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Visualización de Odometría en Tiempo Real', fontsize=16)
        
        # Gráfica 1: Trayectoria X-Y con ejes dinámicos
        self.ax1.set_title('Trayectoria con Ejes Dinámicos (X vs Y)', fontweight='bold')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axis('equal')
        
        # Quitar números de los ejes dinámicos para vista más limpia
        self.ax1.set_xticklabels([])
        self.ax1.set_yticklabels([])
        self.trajectory_line, = self.ax1.plot([], [], 'navy', linewidth=3, alpha=0.8, label='Trayectoria Completa')
        self.current_pos, = self.ax1.plot([], [], 'ro', markersize=12, markeredgecolor='black', markeredgewidth=2, label='Robot Actual')
        
        # Cono de visión para mostrar orientación (mejorado)
        from matplotlib.patches import Polygon
        self.vision_cone = Polygon([(0, 0)], closed=True, alpha=0.5, facecolor='orange', 
                                  edgecolor='red', linewidth=2, label='Campo de Visión')
        self.ax1.add_patch(self.vision_cone)
        
        # Vector de dirección (flecha mejorada)
        self.direction_arrow, = self.ax1.plot([], [], 'red', linewidth=4, alpha=0.9, label='Dirección')
        
        # Información de límites dinámicos
        self.limits_info = self.ax1.text(0.02, 0.02, '', transform=self.ax1.transAxes, fontsize=9,
                                        verticalalignment='bottom', 
                                        bbox=dict(boxstyle='round,pad=0.3', facecolor='lightblue', alpha=0.8))
        
        self.ax1.legend()
        
        # Gráfica 2: X vs Tiempo
        self.ax2.set_title('Posición X vs Tiempo')
        self.ax2.set_xlabel('Tiempo (s)')
        self.ax2.set_ylabel('X (m)')
        self.ax2.grid(True)
        self.x_time_line, = self.ax2.plot([], [], 'g-', linewidth=2)
        
        # Gráfica 3: Y vs Tiempo
        self.ax3.set_title('Posición Y vs Tiempo')
        self.ax3.set_xlabel('Tiempo (s)')
        self.ax3.set_ylabel('Y (m)')
        self.ax3.grid(True)
        self.y_time_line, = self.ax3.plot([], [], 'r-', linewidth=2)
        
        # Gráfica 4: Theta vs Tiempo
        self.ax4.set_title('Orientación Theta vs Tiempo')
        self.ax4.set_xlabel('Tiempo (s)')
        self.ax4.set_ylabel('Theta (rad)')
        self.ax4.grid(True)
        self.theta_time_line, = self.ax4.plot([], [], 'm-', linewidth=2)
        
        # Botón de limpiar datos
        from matplotlib.widgets import Button
        
        # Crear espacio para el botón (ajustar el layout primero)
        plt.tight_layout()
        
        # Ajustar el layout para hacer espacio a los botones
        plt.subplots_adjust(top=0.88)
        
        # Botón de pausa/play (más grande y mejor posicionado)
        pause_ax = plt.axes([0.02, 0.94, 0.08, 0.04])
        self.pause_button = Button(pause_ax, '||', color='lightblue', hovercolor='cornflowerblue')
        self.pause_button.label.set_fontsize(14)
        self.pause_button.label.set_weight('bold')
        self.pause_button.on_clicked(self.toggle_pause)
        
        # Botón de limpiar (más grande y mejor posicionado)
        clear_ax = plt.axes([0.11, 0.94, 0.08, 0.04])
        self.clear_button = Button(clear_ax, 'CLR', color='lightcoral', hovercolor='indianred')
        self.clear_button.label.set_fontsize(12)
        self.clear_button.label.set_weight('bold')
        self.clear_button.on_clicked(self.clear_data)
        
        # Manejador de eventos de teclado para atajos
        def on_key_press(event):
            if event.key == 'c' or event.key == 'C':
                self.clear_data()
            elif event.key == 'r' or event.key == 'R':
                self.clear_data()
            elif event.key == 'p' or event.key == 'P' or event.key == ' ':
                self.toggle_pause()
        
        self.fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    def update_plots(self, frame):
        """Actualiza las gráficas con nuevos datos"""
        with self.data_lock:
            if len(self.x_data) > 0:
                # Convertir tiempo relativo
                if len(self.time_data) > 0:
                    start_time = self.time_data[0]
                    relative_time = [t - start_time for t in self.time_data]
                else:
                    relative_time = []
                
                # Actualizar trayectoria X-Y
                self.trajectory_line.set_data(list(self.x_data), list(self.y_data))
                if len(self.x_data) > 0:
                    self.current_pos.set_data([self.current_x], [self.current_y])
                    
                    # Calcular el cono de visión
                    cone_points = self.calculate_vision_cone(
                        self.current_x, self.current_y, self.current_theta,
                        cone_length=2.0,  # Longitud del cono en metros
                        cone_angle=0.4    # Ángulo de apertura (radianes)
                    )
                    
                    # Actualizar el cono de visión
                    self.vision_cone.set_xy(cone_points)
                    
                    # Vector de dirección (flecha mejorada)
                    arrow_length = 2.5
                    arrow_end_x = self.current_x + arrow_length * np.cos(self.current_theta)
                    arrow_end_y = self.current_y + arrow_length * np.sin(self.current_theta)
                    self.direction_arrow.set_data([self.current_x, arrow_end_x], 
                                                [self.current_y, arrow_end_y])
                    
                    # Actualizar información de límites con datos reales actuales
                    if len(self.x_data) > 0:
                        actual_x_min = min(self.x_data)
                        actual_x_max = max(self.x_data)
                        actual_y_min = min(self.y_data)
                        actual_y_max = max(self.y_data)
                        x_range = actual_x_max - actual_x_min
                        y_range = actual_y_max - actual_y_min
                        
                        limits_text = f"Posición actual: ({self.current_x:.2f}, {self.current_y:.2f})\n"
                        limits_text += f"Orientación: {self.current_theta:.2f} rad ({np.degrees(self.current_theta):.0f}°)\n"
                        limits_text += f"Área explorada: {x_range:.1f}m × {y_range:.1f}m\n"
                        limits_text += f"X: [{actual_x_min:.1f}, {actual_x_max:.1f}] | Y: [{actual_y_min:.1f}, {actual_y_max:.1f}]\n"
                        limits_text += f"Total puntos: {len(self.x_data)}"
                        self.limits_info.set_text(limits_text)
                
                # *** CORRECCIÓN DE EJES DINÁMICOS - CALCULA LÍMITES REALES DE LOS DATOS ***
                if len(self.x_data) > 0:
                    # Calcular límites reales directamente de TODOS los datos actuales
                    actual_x_min = min(self.x_data)
                    actual_x_max = max(self.x_data)  
                    actual_y_min = min(self.y_data)
                    actual_y_max = max(self.y_data)
                    
                    # Calcular rangos con margen proporcional
                    x_range = max(actual_x_max - actual_x_min, 1.0)  # Mínimo 1 metro
                    y_range = max(actual_y_max - actual_y_min, 1.0)  # Mínimo 1 metro
                    
                    # Margen: 20% del rango + mínimo 1 metro para buena visualización
                    margin_x = max(x_range * 0.2, 1.0)
                    margin_y = max(y_range * 0.2, 1.0)
                    
                    # Límites que GARANTIZAN mostrar todos los datos reales
                    new_x_min = actual_x_min - margin_x
                    new_x_max = actual_x_max + margin_x
                    new_y_min = actual_y_min - margin_y
                    new_y_max = actual_y_max + margin_y
                    
                    # Obtener límites actuales del gráfico
                    current_xlim = self.ax1.get_xlim()
                    current_ylim = self.ax1.get_ylim()
                    
                    # Solo expandir si es necesario (nunca contraer)
                    final_x_min = min(current_xlim[0], new_x_min)
                    final_x_max = max(current_xlim[1], new_x_max)
                    final_y_min = min(current_ylim[0], new_y_min)
                    final_y_max = max(current_ylim[1], new_y_max)
                    
                    # DEBUG: Mostrar cuando se expanden los ejes
                    if (actual_x_min < current_xlim[0] or actual_x_max > current_xlim[1] or 
                        actual_y_min < current_ylim[0] or actual_y_max > current_ylim[1]):
                        
                        print(f"🔄 Expandiendo ejes para mostrar datos:")
                        print(f"   Datos reales: X[{actual_x_min:.1f}, {actual_x_max:.1f}] Y[{actual_y_min:.1f}, {actual_y_max:.1f}]")
                        print(f"   Ejes nuevos: X[{final_x_min:.1f}, {final_x_max:.1f}] Y[{final_y_min:.1f}, {final_y_max:.1f}]")
                    
                    # Aplicar límites corregidos que muestran TODOS los datos
                    self.ax1.set_xlim(final_x_min, final_x_max)
                    self.ax1.set_ylim(final_y_min, final_y_max)
                else:
                    # Vista inicial por defecto
                    self.ax1.set_xlim(-2, 2)
                    self.ax1.set_ylim(-2, 2)
                
                # Actualizar gráficas temporales
                if len(relative_time) > 0:
                    self.x_time_line.set_data(relative_time, list(self.x_data))
                    self.y_time_line.set_data(relative_time, list(self.y_data))
                    self.theta_time_line.set_data(relative_time, list(self.theta_data))
                    
                    # Actualizar límites temporales
                    time_min, time_max = min(relative_time), max(relative_time)
                    
                    for ax, data in [(self.ax2, self.x_data), (self.ax3, self.y_data), (self.ax4, self.theta_data)]:
                        if len(data) > 0:
                            data_min, data_max = min(data), max(data)
                            data_margin = (data_max - data_min) * 0.1 or 0.1
                            
                            ax.set_xlim(time_min, time_max + 1)
                            ax.set_ylim(data_min - data_margin, data_max + data_margin)
        
        return (self.trajectory_line, self.current_pos, self.vision_cone, self.direction_arrow,
                self.limits_info, self.x_time_line, self.y_time_line, self.theta_time_line)
    
    def start_visualization(self):
        """Inicia la visualización en tiempo real"""
        print("Iniciando visualización...")
        
        # Configurar gráficas
        self.setup_plots()
        
        # Iniciar thread de lectura serial
        serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        serial_thread.start()
        
        # Configurar animación (más rápida)
        ani = animation.FuncAnimation(
            self.fig, self.update_plots, interval=25, blit=True, cache_frame_data=False
        )
        
        # Mostrar gráficas
        plt.show()
        
        try:
            # Mantener la aplicación corriendo
            while True:
                plt.pause(0.1)
                if not plt.get_fignums():  # Si se cierra la ventana
                    break
        except KeyboardInterrupt:
            print("\nDeteniendo visualización...")
        finally:
            self.stop()
    
    def stop(self):
        """Detiene la visualización y cierra conexiones"""
        print("Cerrando conexiones...")
        self.running = False
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Conexión serial cerrada")
        
        plt.close('all')


def main():
    """Función principal"""
    print("=== VISUALIZADOR DE ODOMETRÍA CON EJES DINÁMICOS ===")
    print("CARACTERÍSTICAS MEJORADAS:")
    print("✓ Lee datos reales del puerto COM6") 
    print("✓ Ejes que se expanden automáticamente")
    print("✓ Vista que SIEMPRE muestra toda la trayectoria")
    print("✓ Cono de visión para orientación")
    print("✓ 4 gráficas simultáneas (X-Y, X-t, Y-t, θ-t)")
    print("\nFORMATO DE DATOS ESPERADO:")
    print("'x:1.23,y:4.56,theta:0.78' por el puerto COM6")
    print("También acepta: 'X:1.23,Y:4.56,THETA:0.78'")
    print("O simplemente: '1.23,4.56,0.78'")
    print("\nPresiona Ctrl+C para detener\n")
    
    # Crear y iniciar el visualizador
    visualizer = OdometriaVisualizer(
        port='COM6',
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