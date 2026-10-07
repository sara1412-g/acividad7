# acividad7
# Enjambre de 3 carritos con algoritmo de hormigas (ACO) en ESP32 y gemelo digital en PyBullet

Tres carritos físicos (nodos ESP32) deben encontrar la ruta óptima desde un punto A hasta una meta en un laberinto/almacén. El algoritmo de optimización por colonia de hormigas (ACO) corre dentro de cada ESP32, los nodos intercambian feromonas por red y el resultado se refleja en una simulación en PyBullet donde 3 robots virtuales replican el comportamiento.

## Arquitectura

```
 MUNDO FÍSICO              PARTE DE RED               PARTE VIRTUAL (Docker)
┌───────────────┐       ┌────────────────┐        ┌─────────────────────────┐
│ 3 × ESP32     │◄─────►│ ESP32 en modo  │◄──────►│ Gemelo digital PyBullet │
│ ACO en cada   │ WiFi  │ AP (red WiFi)  │  UDP   │ Laberinto + 3 carritos  │
│ nodo          │       │ + feromonas    │        │ Visor web :8000         │
└───────────────┘       └────────────────┘        └─────────────────────────┘
```

- **Mundo físico:** cada ESP32 ejecuta su propio ACO y comparte su mejor ruta y sus feromonas.
- **Parte de red:** el nodo 0 funciona en modo AP y crea la red WiFi; los nodos 1 y 2 se conectan a ella. Los datos viajan por UDP.
- **Parte virtual:** un contenedor Docker recibe los datos y los muestra en un gemelo digital hecho con PyBullet.

## Archivos del proyecto

### `aco_swarm.ino`
Firmware de los ESP32. Implementa el ACO, el modo AP (nodo 0), la conexión de los demás nodos y el envío por UDP de la mejor ruta y las feromonas. El nodo se elige con `#define NODE_ID`.

### `aco.py`
Implementación del ACO en Python (misma lógica que el firmware). Aquí están los parámetros del algoritmo.

### `maze.py`
Define el laberinto (`MAZE`), el punto de partida y la meta.

### `emulator.py`
Emula 3 ESP32 virtuales que envían datos por UDP. Permite probar todo sin hardware.

### `twin.py`
Gemelo digital:

- Un hilo escucha UDP en el puerto 4210 y guarda la mejor ruta de cada nodo.
- Con PyBullet construye el laberinto, la salida, la meta y 3 carritos virtuales (uno por nodo).
- Cada carrito recorre la mejor ruta de su nodo y, al llegar, reinicia con la ruta más reciente. Las baldosas se colorean más intensamente donde hay más feromona.
- Publica el video de la cámara de PyBullet en `http://localhost:8000`, por lo que funciona dentro de Docker sin ventana.
- Con `--gui` abre la ventana normal de PyBullet (fuera de Docker).

### `requirements.txt`
Dependencias de Python: `pybullet`, `numpy`, `pillow`.

### `Dockerfile`
Construye la imagen del gemelo digital: parte de Python 3.11, instala el compilador (PyBullet se compila al instalarse), instala las dependencias y ejecuta `twin.py`. La primera construcción puede tardar varios minutos.

### `docker-compose.yml`
Define dos servicios:

- `twin`: el gemelo digital. Expone el puerto 8000 (visor web) y el 4210/UDP (datos de los ESP32).
- `emulator`: 3 ESP32 virtuales. Solo se inicia con `--profile emulator`.

## Cómo usarlo

### Opción A: sin hardware (emulador)

```bash
docker compose --profile emulator up --build
```

Abre <http://localhost:8000> y verás los 3 carritos recorriendo el laberinto mientras la mejor ruta de cada nodo va bajando hasta la ruta óptima de 20 pasos.

### Opción B: con los 3 ESP32

1. En Arduino IDE instala el paquete **esp32 by Espressif** y selecciona la placa `ESP32 Dev Module`.
2. Abre `aco_swarm.ino`, deja `#define NODE_ID 0` y carga la primera placa. Repite con `NODE_ID 1` y `NODE_ID 2` en las otras dos.
3. Enciende primero el nodo 0 (crea el WiFi). Los nodos 1 y 2 se conectan solos.
4. Conecta tu PC a la red WiFi del nodo 0 `ACO_SWARM` (contraseña `aco12345`, definidos en el `.ino`).
5. Ejecuta:

   ```bash
   docker compose up --build
   ```

6. Abre <http://localhost:8000>. Permite el puerto UDP 4210 en el firewall de tu PC.

### Opción C: sin Docker (ventana de PyBullet)

```bash
pip install -r requirements.txt
python twin.py --gui
```

En otra terminal:

```bash
python emulator.py
```

## Parámetros del ACO

Están al inicio de `aco.py` y en `aco_swarm.ino` (deben coincidir en ambos):

| Parámetro | Valor | Significado                                            |
| --------- | ----- | ------------------------------------------------------ |
| `ALPHA`   | 1.0   | Peso de la feromona                                    |
| `BETA`    | 1.0   | Peso de la cercanía a la meta                          |
| `RHO`     | 0.10  | Evaporación por iteración                              |
| `Q`       | 10    | Feromona depositada (Q / largo de la ruta)             |
| `ANTS`    | 4     | Hormigas por iteración                                 |

Para cambiar el laberinto, edita `MAZE` en `maze.py` y el mismo arreglo (junto con `R` y `C`) en el `.ino`.

## Solución de problemas

- **El visor dice "Esperando datos":** el contenedor no está recibiendo UDP. Docker Desktop a veces no reenvía broadcast; ejecuta `python twin.py` directamente en el PC o, en Linux, usa `network_mode: host` en `docker-compose.yml`.
- **Un ESP32 no se conecta:** revisa que el nodo 0 esté encendido y que SSID y contraseña coincidan en los tres.
- **La vista aparece rotada:** en `twin.py` cambia el yaw `0` por `90` en `computeViewMatrixFromYawPitchRoll`.

## Autores

- Nombre Apellido – [@usuario](https://github.com/usuario)

Materia: Microcontroladores · Actividad 7
