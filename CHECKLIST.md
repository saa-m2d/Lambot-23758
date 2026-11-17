# Furby v1 — Checklist técnico (Fase 1)

Robot de LamBot 23758 para BIOBUZZ (FTC 2026-2027). SDK 12.0, PedroPathing 3.0.2 instalado (sin tunear).

Esta fase solo tiene pruebas. Su objetivo es confirmar el cableado y medir lo que el resto del
código necesita. Al final hay una hoja de resultados: llénala y con eso se escribe la Fase 2.

## 1. Configuración en el Driver Hub

Los nombres deben quedar **idénticos**, con mayúsculas y minúsculas. Están definidos en
`TeamCode/.../config/RobotMap.java`.

### Motores (tipo: goBILDA 5202/5203/5204 series)

| Hub | Puerto | Nombre | Motor |
|---|---|---|---|
| Control Hub | 0 | `shooterLeft` | Yellow Jacket 6000 RPM, con cable de encoder |
| Control Hub | 1 | `shooterRight` | Yellow Jacket 6000 RPM, con cable de encoder |
| Control Hub | 2 | `frontLeft` | 1620 RPM |
| Control Hub | 3 | `backLeft` | 1620 RPM |
| Expansion Hub | 0 | `intakeRight` | 1620 RPM |
| Expansion Hub | 1 | `intakeLeft` | 1620 RPM |
| Expansion Hub | 2 | `backRight` | 1620 RPM |
| Expansion Hub | 3 | `frontRight` | 1620 RPM |

- [ ] Los 8 motores están con ese nombre y en ese puerto.
- [ ] `intakeLeft` está en Expansion Hub 1. En la lista original, "Intake Right" aparecía dos veces
      (puertos 0 y 1); aquí se asumió que el del puerto 1 es el izquierdo. **Confirmar.**

### Servos

| Hub | Puerto | Nombre | Servo | Tipo en el Driver Hub |
|---|---|---|---|---|
| Control Hub | 4 | `turretLeft` | Axon MINI MK2, firmware CR | Continuous Rotation Servo |
| Control Hub | 5 | `flowerLeft` | Swyft speed servo | Servo |
| Expansion Hub | 0 | `tope` | Axon MINI MK2, modo posición | Servo |
| Expansion Hub | 1 | `turretRight` | Axon MINI MK2, firmware CR | Continuous Rotation Servo |
| Expansion Hub | 2 | `flowerRight` | Swyft speed servo | Servo |

- [ ] Los dos servos de la torreta ya tienen el firmware CR (ver sección 5) y están como
      **Continuous Rotation Servo**. Si los Swyft resultan ser continuos, también se cambian.

### Sensores

| Dónde | Nombre | Tipo |
|---|---|---|
| Expansion Hub, I2C bus 0 | `pinpoint` | goBILDA Pinpoint Odometry Computer |
| Control Hub, analógico 0 | `turretEnc` | Analog Input (4.º cable de `turretLeft`) |
| Control Hub, USB 3.0 (azul) | `limelight` | Limelight 3A (aparece con Scan) |

- [ ] Pinpoint configurado. Su dirección I2C es fija (0x31); no hay nada que cambiar.
- [ ] Limelight conectada con cable USB-A a USB-C al puerto **azul**. No usar el USB 2.0.

## 2. Servos: modos y PWM

El equipo tiene el programador Axon MK2 (`Axon_Software_MK2_v1.1.2.exe`, en Descargas; no se
instala, se abre directo). El modo se cambia cargando firmware: `Axon_MINI_MK2_Servo_Mode.sfm` o
`Axon_MINI_MK2_CR_Mode.sfm`.

- **Modo posición** (tope): el código manda **500 a 2500 µs** (posición 0.0 a 1.0). De fábrica
  recorre unos 350° en ese rango (0.175° por µs).
- **Modo continuo** (torreta): el código manda **1050 a 1950 µs** (potencia -1 a 1); 1500 µs es
  detenido. Si la torreta se mueve sola sin potencia, se corrige con **Neutral**.
- **Tope**, medido con el programador y configuración de fábrica (L 250, R 250, Neutral 0):
  abierto = **1802 µs** (posición 0.651), cerrado = **2300 µs** (posición 0.900).
- De los Swyft no hay datos confirmados. La prueba 2 dice cómo se comportan.

## 3. Pinpoint

- Pods: goBILDA 4-Bar.
- Offsets, ya cargados en `RobotMap.java`:
  - Pod X (forward): **-82.5 mm** (a la derecha del centro).
  - Pod Y (strafe): **-70.74 mm** (atrás del centro).
- Las direcciones de los pods se confirman en la prueba 3.

## 4. Limelight (interfaz web, con la cámara conectada a una laptop)

- [ ] Firmware actualizado y número de equipo 23758.
- [ ] Pipeline 0 de tipo AprilTag, familia 36h11, tamaño de tag **82.5 mm** (3.25 in).
- [ ] Mapa de campo de BIOBUZZ (.fmap) cargado. Sin él no hay MegaTag.
- [ ] Posición de la cámara: 9.5 cm atrás del eje de la torreta, centrada, inclinada 54° hacia
      arriba. **Falta medir la altura de la lente sobre el piso.**

## 5. Torreta: modo continuo con encoder

Home = la torreta apuntando a la **izquierda** del robot (90° antihorario desde el frente).
Rango con la cadena portacables: **360° antihorario y 40° horario** desde home.

Antes de la prueba 5:

1. Con el programador, **Save** de cada servo de la torreta (respaldo) y luego carga
   `Axon_MINI_MK2_CR_Mode.sfm` en los dos. Deben quedar con la misma configuración.
2. Revisa que **Lose PPM Protection** esté en **Release** en los dos: así el servo sin señal queda
   suelto y no frena al otro.
3. Conecta el 4.º cable de `turretLeft` al analógico 0 del Control Hub (puede necesitar el
   adaptador JST de Axon).
4. Gira la torreta a mano por todo el rango: la cadena no debe tensarse. Marca los dos límites y
   la posición de home con cinta.
5. Al engranar, procura que **en home el voltaje del encoder quede cerca de 1.65 V** (la mitad).
   Así el robot sabe su ángulo exacto aunque arranque hasta ±90° fuera de home.

**No uses la prueba 2 con la torreta**: esa prueba trata a los servos como de posición, y en
modo continuo los haría girar sin límites. La prueba 5 empieza con un solo servo, limita el giro
a ±30° hasta confirmar direcciones y corta la potencia si la torreta se mueve al revés.

El procedimiento paso a paso está en el comentario inicial de `TurretTestOpMode.java`. Los
resultados se copian a `config/TurretConfig.java`.

## 6. Orden de las pruebas

Compilar y cargar desde Android Studio, o con:

```bash
./gradlew :TeamCode:assembleDebug
```

| # | OpMode | Preparación |
|---|---|---|
| 1 | Test 1: Motores | Robot sobre bloques, sin pelotas |
| 2 | Test 2: Servos | Manos lejos; todo empieza suelto |
| 3 | Test 3: Pinpoint | Robot en el piso, se empuja a mano |
| 4 | Test 4: Limelight | Torreta quieta, un tag a la vista |
| 5 | Test 5: Torreta | Firmware CR y encoder conectado; torreta en la marca de home |

Los controles de cada prueba están en el comentario inicial de su archivo, en
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/`.

## 7. Hoja de resultados

**Motores** (con potencia positiva, ¿hacia dónde gira?)

| Motor | Barrido OK | Sentido con potencia + |
|---|---|---|
| frontLeft | | empuja el robot adelante / atrás |
| backLeft | | adelante / atrás |
| frontRight | | adelante / atrás |
| backRight | | adelante / atrás |
| shooterLeft | | lanza / regresa |
| shooterRight | | lanza / regresa |
| intakeLeft | | mete / saca. ¿Es el de enfrente o el de atrás? |
| intakeRight | | mete / saca. ¿Es el de enfrente o el de atrás? |

**Servos**

| Servo | ¿Posición o continuo? | Valores |
|---|---|---|
| tope | posición | abierto: 0.651 (1802 µs) cerrado: 0.900 (2300 µs) |
| flowerLeft | | guardado: ____ extrayendo: ____ |
| flowerRight | | guardado: ____ extrayendo: ____ |

**Torreta** (prueba 5; copiar a `TurretConfig.java`)

- `LEFT_REVERSED`: true / false. `RIGHT_REVERSED`: true / false. `ENCODER_REVERSED`: true / false
- `ENCODER_V_MIN`: ____ V. `ENCODER_V_MAX`: ____ V
- `HOME_VOLTAGE`: ____ V (ideal: cerca de 1.65)
- ¿Gira suave con los dos servos, sin tirones ni ruido?: sí / no
- ¿Se mueve sola con potencia 0? (si sí, ajustar Neutral): sí / no
- ¿Los límites del código (-40° y 360°) coinciden con las marcas físicas?: sí / no

**Pinpoint**

- Estado READY: sí / no
- Dirección pod X: FORWARD / REVERSED. Dirección pod Y: FORWARD / REVERSED
- Tras 10 vueltas sobre su centro, X y Y regresaron a: ____ , ____ cm

**Limelight**

- Conexión y fps: ____
- Tags que ve desde la posición de inicio: ____
- ¿MegaTag 2 se mantiene estable al girar el chasis?: sí / no
- Altura de la lente sobre el piso: ____ cm
