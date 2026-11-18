# Delta-SC-02 en un ESP32-C3

Cabezal de lidar sacado de una Roomba RCC-Y2. La placa de la aspiradora no está. El módulo es un Delta-SC-02 (D_V01, 2024-08-31) y no obedece órdenes: con la lógica alimentada, escupe tramas por TX1 en cuanto el motor lo hace girar.

El C3 hace tres cosas. Baja esos 5 V a la entrada de 3,3 V, limita el motor de 3,5 V colgado de una alimentación de 5 V, y usa un byte de la propia trama para mantener las revoluciones. El USB lleva el flujo binario tal cual. ROS no entra en esto.

## El apaño

El conector J1, de arriba abajo, es M+, M−, 5 V, GND, TX1. Los pads VPP, PDA y PCL son de programación; no se usan.

TX1 va a 5 V. El UART del C3 no. Entre medias:

```
TX1 ---- 1 kΩ ---- GPIO20 ---- 2 kΩ ---- GND
```

En GPIO20 quedan unos 3,3 V. GPIO21 se deja suelto: el cabezal no tiene RX. GPIO18 y GPIO19 son D− y D+ del USB.

La fuente es un buck-boost a 5 V y alimenta la lógica por el pin de 5 V. El motor, en cambio, es de 3,5 V. M+ va a esos 5 V y M− al lado bajo, gobernado por GPIO4. El firmware no deja pasar el duty del 70 %, porque 0,70 × 5 V = 3,5 V. `MAX` puede subirlo, y entonces el motor ve más de eso.

Un PWM fijo no mantiene las vueltas: cambian el rozamiento y la tensión. Dentro de cada trama válida, el byte de offset 8 baja cuando el cabezal frena. El C3 lo lee como Hz = byte / 20 y mueve el duty con un PI (kp 8, ki 5, objetivo 6 Hz). Los bytes que salen por USB son los de TX1; el lazo solo mira una copia.

Si durante 800 ms no entra ninguna trama con checksum bueno, el integrador se tira y el duty vuelve a `START` (50 %). Así un cabezal callado no se queda al 70 %.

## Qué se considera una trama

Hace falta esto, y el resto del paquete se reenvía sin mirarlo:

```
AA  LL LL  ..  61  AD|AE  ..  VV  ..  SS SS
0   1  2   3   4   5       8
```

`LL` es la longitud en 16 bits, byte alto primero, contada desde `AA` hasta el byte anterior al checksum. `61` y `AD` (distancias) o `AE` (solo velocidad) tienen que estar en su sitio. `SS` es la suma de esos `LL` bytes, también en 16 bits y byte alto primero. `VV` es el byte de velocidad.

El 20 de `VV / 20` es el factor con el que este cabezal cuadra con otros Delta de la misma cabecera. Si las vueltas reales no coinciden con lo que imprime el C3, se cambia con `SCALE` sin recompilar. Las distancias no se descodifican aquí: `sonar.py` y `listen.py` aplican raw × 0,25 mm y ángulo × 0,01°, escalas de esos otros Delta, no de un datasheet de este SC-02.

## El mismo cable

A 115200, 8N1. Por USB salen los bytes del lidar y, cuando el lazo habla, una línea de texto que empieza por `##`. Quien parsee tiene que saltársela. Por USB entran líneas terminadas en `\n`; no se reenvían a TX1.

| Línea | Efecto |
| --- | --- |
| `HZ 6` | Objetivo del PI, de 0 a 20. Enciende `AUTO`. |
| `PWM 40` | Duty fijo. Apaga el PI. |
| `AUTO 1` / `AUTO 0` | PI o duty fijo. |
| `MAX 70` / `MIN 0` | Techo y suelo del duty. 70 es el tope del motor. |
| `START 50` | Duty de arranque y de recuperación si se pierden tramas. |
| `FREQ 1000` | Hz del PWM, entre 50 y 20000. |
| `SCALE 20` | Divisor del byte de velocidad. |
| `KP` `KI` `KD` | Ganancias. `KD` arranca en 0. |
| `STATUS` | Imprime el estado y no lo repite. |
| `TICK 1` | Una línea `##` por segundo. `TICK 0` lo quita. |

Al abrir el puerto hay que dejar DTR y RTS bajos. Si no, el C3 se reinicia al modo de programación. `listen.py` y `sonar.py` ya lo hacen.

## Uso

```
cd firmware && pio run -t upload
pip install pyserial
python3 sonar.py
python3 listen.py stats
./check_c3_pc.sh
```

Placa `esp32-c3-devkitm-1`, USB nativo, puerto `/dev/ttyACM0`. Hace falta estar en el grupo `dialout`. `sonar.py` es el PPI y manda `HZ 6` y `AUTO 1`. `listen.py` guarda el crudo en `captures/`.
