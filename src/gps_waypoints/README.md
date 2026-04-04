GPS Waypoints ROS2 node

Este paquete proporciona un nodo Python que lee un archivo JSON con checkpoints (lat, lon, alt) y publica el checkpoint actual como un mensaje `sensor_msgs/NavSatFix` hasta que el GPS real alcance la coordenada (basado en un umbral en metros).

Uso rápido:

1. Construir el workspace:

```bash
colcon build --packages-select gps_waypoints
source install/setup.bash
```

2. Ejecutar el nodo (ajusta parámetros con `--ros-args -p`):

```bash
ros2 run gps_waypoints gps_waypoint_node --ros-args -p checkpoints_file:=$(pwd)/src/gps_waypoints/checkpoints/sample_checkpoints.json -p threshold_m:=5.0
```

Parámetros importantes:
- `gps_topic` (por defecto `/wamv/sensors/gps/gps/fix`)
- `goal_topic` (por defecto `/waypoint_gps`) — el nodo publica objetivos GPS aquí
- `checkpoints_file` — ruta al JSON de checkpoints
- `threshold_m` — distancia en metros para considerar alcanzado un checkpoint

Formato JSON aceptado:

Lista directa o objeto con key `checkpoints`. Ejemplo:

```json
{
  "checkpoints": [
    {"lat": 37.4275, "lon": -122.1697, "alt": 0},
    {"lat": 37.4280, "lon": -122.1705}
  ]
}
```

El nodo evita dependencias externas y publica las coordenadas GPS objetivo; si tu flight stack necesita objetivos en coordenadas locales, añade un convertidor de GPS->ENU que consuma `goal_topic`.
