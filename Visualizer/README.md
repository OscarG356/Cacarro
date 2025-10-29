# Visualizador de Odometría en Tiempo Real

Este proyecto permite leer datos de odometría del puerto serial COM4 y visualizarlos en tiempo real mediante gráficas interactivas.

## Características

- **Lectura Serial**: Lee datos del puerto COM4 en tiempo real
- **Visualización Múltiple**: 4 gráficas simultáneas:
  - Trayectoria X-Y (vista cenital del movimiento)
  - Posición X vs Tiempo
  - Posición Y vs Tiempo  
  - Orientación Theta vs Tiempo
- **Formato Flexible**: Acepta diferentes formatos de datos
- **Buffer Configurable**: Almacena historial de datos para análisis
- **Thread-Safe**: Lectura serial y visualización en hilos separados

## Requisitos

- Python 3.7+
- Librerías: `pyserial`, `matplotlib`, `numpy`

## Formato de Datos Esperado

El programa espera datos en el puerto COM4 con uno de estos formatos:

```
x:1.234,y:5.678,theta:0.789
X:2.345,Y:6.789,THETA:1.234
1.111,2.222,3.333
```

Donde:
- **x, y**: Coordenadas de posición (metros)
- **theta**: Orientación angular (radianes)

## Instalación y Uso

### 1. Configuración del Entorno

Las librerías ya están instaladas en el entorno virtual del proyecto.

### 2. Ejecución

```powershell
# Activar entorno virtual (si es necesario)
.\.venv\Scripts\Activate.ps1

# Ejecutar el visualizador principal
python odometria_visualizer.py
```

### 3. Configuración del Puerto

Por defecto usa COM4 a 9600 baudios. Para cambiar:

```python
# En odometria_visualizer.py, línea ~290
visualizer = OdometriaVisualizer(
    port='COM5',      # Cambiar puerto
    baudrate=115200,  # Cambiar velocidad
    buffer_size=300   # Cambiar tamaño del buffer
)
```

## Pruebas sin Hardware

### Simulación de Datos

```powershell
# Generar datos de prueba
python simulation_utils.py

# Simulación continua
python simulation_utils.py --simulate
```

### Prueba de Parseo

```powershell
python simulation_utils.py
```

## Estructura del Código

### `odometria_visualizer.py`
- **OdometriaVisualizer**: Clase principal del visualizador
- **connect_serial()**: Establece conexión con puerto COM
- **parse_serial_data()**: Interpreta formatos de datos
- **read_serial_data()**: Hilo de lectura continua
- **setup_plots()**: Configura las 4 gráficas
- **update_plots()**: Actualiza visualización en tiempo real

### `simulation_utils.py` 
- **OdometryDataGenerator**: Generador de datos sintéticos
- **simulate_odometry_data()**: Simulación de puerto serial
- **test_data_parsing()**: Pruebas de formato de datos

## Configuraciones Avanzadas

### Cambiar Velocidad de Actualización

```python
# En start_visualization(), cambiar interval
ani = animation.FuncAnimation(
    self.fig, self.update_plots, 
    interval=100,  # Milisegundos (100ms = 10 FPS)
    blit=False
)
```

### Modificar Tamaño del Buffer

```python
visualizer = OdometriaVisualizer(
    buffer_size=500  # Más puntos en historial
)
```

### Personalizar Gráficas

Editar `setup_plots()` para:
- Cambiar colores de líneas
- Modificar títulos y etiquetas
- Ajustar estilos de marcadores
- Añadir leyendas personalizadas

## Solución de Problemas

### Error de Puerto Serial
```
Error al conectar con COM4: [Errno 2] No such file or directory: 'COM4'
```
**Solución**: Verificar que el dispositivo esté conectado y el puerto sea correcto.

### Datos No Reconocidos
```
Error parseando datos: invalid_data - ...
```
**Solución**: Verificar que los datos sigan el formato esperado.

### Ventana No Responde
- Usar Ctrl+C en terminal para detener
- Cerrar ventana de gráficas manualmente

## Extensiones Posibles

1. **Guardar Datos**: Exportar trayectoria a CSV/JSON
2. **Filtros**: Implementar filtro Kalman para suavizar datos
3. **Calibración**: Añadir corrección de deriva y offset
4. **Múltiples Robots**: Visualizar varios robots simultáneamente
5. **Mapeo**: Integrar con datos de sensores para SLAM

## Ejemplo de Uso Típico

```python
from odometria_visualizer import OdometriaVisualizer

# Configurar visualizador
viz = OdometriaVisualizer(
    port='COM4',
    baudrate=9600,
    buffer_size=200
)

# Iniciar visualización
viz.start_visualization()
```

El programa mostrará 4 ventanas gráficas actualizándose en tiempo real conforme lleguen datos por el puerto serial.