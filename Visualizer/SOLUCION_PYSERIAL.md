# 🔧 Guía de Soluciones para Problemas con PySerial

## ❌ Problema Común
```
ModuleNotFoundError: No module named 'serial'
```

## ✅ Soluciones Disponibles

### 1. 📋 Verificar Instalación de PySerial

#### Windows PowerShell:
```powershell
# Activar entorno virtual
C:\Users\oscar\Downloads\Robotica\Odometria\.venv\Scripts\Activate.ps1

# Verificar instalación
python -c "import serial; print('PySerial funciona correctamente')"

# Si falla, reinstalar:
pip uninstall pyserial
pip install pyserial
```

#### Comando directo:
```powershell
C:\Users\oscar\Downloads\Robotica\Odometria\.venv\Scripts\python.exe -m pip install pyserial
```

### 2. 🚀 Usar Visualizador Sin PySerial

```powershell
# Ejecutar visualizador que no requiere pyserial
python visualizador_sin_serial.py
```

**Opciones disponibles:**
- ✅ Datos simulados (recomendado para pruebas)
- 📄 Leer desde archivo de texto
- ⌨️ Entrada manual de datos

### 3. 🌐 Usar Visualizador por Socket

```powershell
# Opción 1: Servidor (recibe datos)
python visualizador_socket.py
# Seleccionar opción 1

# Opción 2: Cliente de prueba (envía datos)
python visualizador_socket.py  
# Seleccionar opción 2
```

**Para enviar datos reales:**
```bash
# Desde otra terminal o dispositivo:
telnet localhost 8888
# Luego enviar: x:1.5,y:2.3,theta:0.785
```

### 4. 🔍 Diagnóstico Completo

```powershell
# Verificar Python
python --version

# Verificar pip
pip --version

# Listar paquetes instalados
pip list | findstr serial

# Verificar ruta del entorno
python -c "import sys; print(sys.executable)"

# Probar importación
python -c "
try:
    import serial
    print('✅ PySerial disponible')
    print(f'Versión: {serial.__version__}')
    print(f'Ubicación: {serial.__file__}')
except ImportError as e:
    print(f'❌ Error: {e}')
"
```

## 📋 Formatos de Datos Soportados

### Para COM Serial:
```
x:1.5,y:2.3,theta:0.785
```

### Para Socket:
```
x:1.5,y:2.3,theta:0.785
{"x": 1.5, "y": 2.3, "theta": 0.785}
1.5,2.3,0.785
```

### Para Archivo:
```
# archivo_datos.txt
x:0.0,y:0.0,theta:0.0
x:1.2,y:0.5,theta:0.1
x:2.1,y:1.2,theta:0.3
```

## 🛠 Comandos de Emergencia

### Reinstalación completa:
```powershell
# Eliminar entorno virtual
Remove-Item -Recurse -Force .venv

# Crear nuevo entorno
python -m venv .venv

# Activar
.venv\Scripts\Activate.ps1

# Instalar dependencias
pip install matplotlib numpy pyserial
```

### Verificación de funcionamiento:
```powershell
# Probar visualizador básico
python demo_simulado.py

# Probar con datos dinámicos
python demo_ejes_dinamicos.py

# Probar versión ultra-rápida
python demo_ultra_rapido.py
```

## 📂 Archivos Disponibles

| Archivo | Descripción | Requiere PySerial |
|---------|-------------|-------------------|
| `odometria_visualizer.py` | ✅ Versión completa con COM | ✅ Sí |
| `visualizador_sin_serial.py` | 🚀 Sin dependencias de serial | ❌ No |
| `visualizador_socket.py` | 🌐 Comunicación por red | ❌ No |
| `demo_simulado.py` | 🎮 Datos simulados | ❌ No |
| `demo_ejes_dinamicos.py` | 📈 Ejes que se expanden | ❌ No |
| `demo_ultra_rapido.py` | ⚡ Optimizado para velocidad | ❌ No |

## ⚡ Inicio Rápido

### Para COM Serial (si funciona):
```powershell
python odometria_visualizer.py
```

### Para pruebas sin hardware:
```powershell
# Opción más simple
python visualizador_sin_serial.py

# Seleccionar opción 1 (datos simulados)
```

### Para datos remotos:
```powershell
# Terminal 1: Servidor
python visualizador_socket.py
# Seleccionar opción 1

# Terminal 2: Cliente de prueba
python visualizador_socket.py
# Seleccionar opción 2
```

## 🎯 Recomendaciones

1. **Para desarrollo/pruebas**: `visualizador_sin_serial.py`
2. **Para datos reales COM**: `odometria_visualizer.py` 
3. **Para datos remotos**: `visualizador_socket.py`
4. **Para demos**: Cualquier archivo `demo_*.py`

## 📞 Soporte Técnico

Si ninguna opción funciona:
1. Verificar versión de Python (3.7+)
2. Verificar que matplotlib funciona: `python -c "import matplotlib.pyplot as plt; print('OK')"`
3. Usar versión sin dependencias: `visualizador_sin_serial.py`
4. Reportar error específico con: `python --version` y `pip list`

---

## 🎮 Ejemplos de Uso

### Datos manuales:
```
python visualizador_sin_serial.py
# Seleccionar opción 3
# Ingresar: 1.0,2.0,1.57
```

### Desde archivo:
```
# Crear archivo_test.txt con:
x:0,y:0,theta:0
x:1,y:1,theta:0.5
x:2,y:2,theta:1.0

python visualizador_sin_serial.py
# Seleccionar opción 2
# Archivo: archivo_test.txt
```