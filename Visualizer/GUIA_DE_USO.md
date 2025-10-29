# 🚀 Visualizador de Odometría - Guía de Uso

## 📁 Archivos Disponibles

### 1. **`demo_ejes_dinamicos.py`** ⭐ **RECOMENDADO PARA TU PROBLEMA**
```bash
python demo_ejes_dinamicos.py
```
**✅ RESUELVE TU PROBLEMA:**
- Los ejes se expanden automáticamente 
- Los ejes **NUNCA se contraen**
- **SIEMPRE** puedes ver toda la trayectoria
- Movimientos claros en X, Y y theta
- Cono de visión que muestra orientación

**📊 Lo que verás:**
- Fase 1: Movimiento recto hacia ESTE (X de 0 a +12m)
- Fase 2: Giro en el lugar (theta cambia, X,Y no)
- Fase 3: Movimiento hacia NORTE (Y de 0 a +10m)  
- Fase 4: Giro 180°
- Fase 5: Movimiento hacia SUR (Y baja)
- Fase 6: Círculo amplio

### 2. **`demo_ultra_rapido.py`** - Ultra Rápido
```bash
python demo_ultra_rapido.py
```
- 50 FPS rendering
- Tema oscuro espectacular
- Vista optimizada

### 3. **`demo_cono_vision.py`** - Cono de Visión Detallado
```bash
python demo_cono_vision.py
```
- Múltiples patrones de movimiento
- Cono de visión mejorado
- Información detallada

### 4. **`odometria_visualizer.py`** - Para Datos Reales
```bash
python odometria_visualizer.py
```
- Lee datos del puerto COM4
- Para usar con tu hardware real
- Formato: `x:1.23,y:4.56,theta:0.78`

### 5. **`demo_simulado.py`** - Demo Original
```bash
python demo_simulado.py
```
- Versión original básica

## 🎯 **Para tu Problema Específico**

**Usa `demo_ejes_dinamicos.py`** porque:

✅ **Ejes dinámicos**: Se expanden automáticamente  
✅ **Vista completa**: Siempre ves toda la trayectoria  
✅ **No se contrae**: Los límites solo crecen, nunca se reducen  
✅ **Movimientos claros**: X, Y y theta cambian notoriamente  
✅ **Cono de visión**: Muestra orientación del robot  

## 🚀 Ejecución Rápida

```powershell
# Para resolver tu problema específico
python demo_ejes_dinamicos.py

# Para datos reales del COM4
python odometria_visualizer.py

# Para rendimiento ultra-rápido
python demo_ultra_rapido.py
```

## 📈 Qué Esperar

El `demo_ejes_dinamicos.py` te mostrará:

1. **Robot rojo** con punto actual
2. **Trayectoria azul** completa (histórica)  
3. **Cono naranja** mostrando orientación
4. **Flecha roja** indicando dirección
5. **Ejes que crecen** automáticamente
6. **Información en tiempo real**: posición, orientación, fase

**Los ejes comenzarán pequeños y se irán expandiendo conforme el robot se mueva, ¡pero nunca se contraerán!**

## 🔧 Personalización

Para cambiar velocidad de animación:
```python
# En cualquier demo, cambiar interval
ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50)  # más rápido
ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100) # más lento
```

Para cambiar tamaño del cono:
```python
self.cone_length = 3.0  # más largo
self.cone_angle = 0.5   # más ancho
```