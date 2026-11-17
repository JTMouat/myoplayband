# MyoPlayBand · Panel en vivo

Panel web para monitorear en tiempo real la **MyoPlayBand**, una banda con ESP32-S3
pensada para apoyar la rehabilitación de miembro superior post-ACV.

🔗 **Página:** https://jtmouat.github.io/myoplayband/

## Qué hace
- Muestra un monito 2D que sigue el movimiento real del brazo (MPU6050).
- Grafica ángulo, velocidad angular y señal muscular (EMG) en tiempo real.
- Evalúa cada repetición: rango, velocidad, mantención, activación y relajación muscular.
- Permite calibrar la placa y reiniciar el contador de forma remota.
- Incluye 3 ejercicios: flexo-extensión de codo, extensión de muñeca y flexión de hombro.

## Cómo funciona
ESP32-S3 → broker MQTT público (`broker.emqx.io`) → esta página (MQTT sobre WebSocket).

## Uso
1. Abrir **broker.emqx.io**.
2. **New Subscription** y dejar **myoplayband/grupo8_basti_jt/#** en el topic.
3. Abrir la GitHub-Page expuesta arriba y elegir **En vivo (MQTT)**.
4. Apretar **Conectar**.
5. Encender la banda y calibrar con el brazo en la posición inicial.

También tiene un modo **Demostración** y la opción **Simular placa** para probar sin hardware.

## Equipo
Grupo 8, Proyecto de Diseño Eléctrico 2026_20 — Universidad de los Andes.
