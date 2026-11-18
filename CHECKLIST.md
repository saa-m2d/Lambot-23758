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
| Control Hub, analógico 0 (conector 0-1, pin azul) | `turretEncLeft` | Analog Input (4.º cable de `turretLeft`) |
| Expansion Hub, analógico 2 (conector 2-3, pin azul) | `turretEncRight` | Analog Input (4.º cable de `turretRight`) |
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
- **Tope** (configuración de fábrica: L 250, R 250, Neutral 0), medido en el robot con la
  prueba 2: abierto = **1900 µs** (posición 0.700), cerrado = **2295 µs** (posición 0.8975).
  Con el programador había salido abierto 1802 µs; manda la medida en el robot.
- **Swyft (flower)**: en modo posición. Valores en la hoja de resultados.

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
3. Conecta el 4.º cable de cada servo: `turretLeft` al analógico 0 del Control Hub y
   `turretRight` al analógico 2 del Expansion Hub (en ambos, el pin del cable azul). Con los dos,
   el código compara las lecturas y detecta si un engrane patina o un cable falla.
4. Gira la torreta a mano por todo el rango: la cadena no debe tensarse. Marca los dos límites y
   la posición de home con cinta.
5. El robot sabe su ángulo exacto si arranca a menos de ±90° de home, sin importar en qué voltaje
   quede home. Aun así, si se puede, conviene que en home cada encoder **no** quede justo en el
   salto de 3.3 V a 0 V, donde la lectura puede ser inestable.

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
| 5 | Test 5: Torreta | Firmware CR y encoders conectados; torreta en la marca de home |
| 6 | Test 6: Tiro | Pruebas 1-5 listas; robot viendo la CELL; pelotas POLLEN |

Los controles de cada prueba están en el comentario inicial de su archivo, en
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/`.

## 7. Hoja de resultados

**Motores** (con potencia positiva, ¿hacia dónde gira?)

Sentidos medidos el 2026-11-17 y ya copiados a `RobotMap.java` (`*_DIRECTION`).

| Motor | Barrido OK | Sentido con potencia + | Dirección en código |
|---|---|---|---|
| frontLeft | | atrás | REVERSE |
| backLeft | | atrás | REVERSE |
| frontRight | | adelante | FORWARD |
| backRight | | adelante | FORWARD |
| shooterLeft | | regresa | REVERSE |
| shooterRight | | regresa | REVERSE |
| intakeLeft | | mete. ¿Es el de enfrente o el de atrás? | FORWARD |
| intakeRight | | mete. ¿Es el de enfrente o el de atrás? | FORWARD |

**Servos**

| Servo | ¿Posición o continuo? | Valores |
|---|---|---|
| tope | posición | abierto: 0.700 (1900 µs) cerrado: 0.8975 (2295 µs) |
| flowerLeft | posición | arriba: 0.830 (2160 µs) abajo: 0.4875 (1475 µs) |
| flowerRight | posición | arriba: 0.200 (900 µs) abajo: 0.490 (1480 µs) |

- Los dos flower mueven la misma pieza, montados en espejo: cada uno se calibra solo (con el
  otro suelto) y giran en sentidos contrarios.
- flowerRight abajo se midió primero en 0.750 (solo, empujaba contra el tope); con los dos
  juntos zumbaba y se corrigió a 0.490.
- [ ] En la prueba 2, modo "flower juntos" (botón X / □): los dos energizados, recorrido
      completo de arriba a abajo con el stick, y ninguno zumba en ningún punto.

**Torreta** (prueba 5; copiar a `TurretConfig.java`)

- `LEFT_REVERSED`: true / false. `RIGHT_REVERSED`: true / false
- Encoder izquierdo: `LEFT_ENCODER_REVERSED` true / false, V mín ____ V, V máx ____ V,
  `LEFT_HOME_VOLTAGE` ____ V
- Encoder derecho: `RIGHT_ENCODER_REVERSED` true / false, V mín ____ V, V máx ____ V,
  `RIGHT_HOME_VOLTAGE` ____ V
- ¿Los dos encoders coinciden al recorrer todo el rango? Diferencia máxima: ____°
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

## 8. Código de competencia (Fase 2)

Ya está escrito todo: `subsystems/`, `commands/`, `diagnostics/`, `Robot.java`, el TeleOp
**Furby TeleOp** y los autónomos **Furby Auto ROJO / AZUL**. Los controles están en el comentario
inicial de `opmodes/teleop/CompetitionTeleOp.java`.

### Panels (dashboard)

Con la laptop en la red del robot: **http://192.168.43.1:8001**. Ahí se ve la telemetría y se
cambian en vivo las constantes de `config/*Constants.java` (RPM, PID de la torreta, etc.). Lo que
se cambia en Panels se pierde al reiniciar: los valores buenos se copian al código.

### Cómo apunta (importante)

Los tags de BIOBUZZ están en las CELLS, que se voltean; el SDK no les da posición fija en el campo.
Por eso el robot no se ubica con los tags para apuntar: cuando la Limelight ve la CELL de arriba,
calcula dónde está respecto al robot y la guarda con el Pinpoint. Si deja de verla, sigue
apuntando con el Pinpoint. MegaTag 2 queda disponible pero apagado (`VisionConstants.USE_MEGATAG`).

### Falta medir o llenar

- [ ] `VisionConstants.TURRET_AXIS_FORWARD_IN` / `TURRET_AXIS_LEFT_IN`: dónde está el eje de la
      torreta respecto al centro del robot (pulgadas).
- [ ] Test 4: con un tag a distancia conocida, "desde la torreta → distancia" debe coincidir con la
      cinta y el ángulo debe ser casi `-tx`. Si no, revisar `CAMERA_PITCH_DEG` y `CAMERA_BEHIND_AXIS_IN`.
- [ ] Test 6: tabla de tiro (`ShooterConstants.TABLE_*`) desde 6 a 10 distancias.
- [ ] `ShooterConstants.TOF_SECONDS`: tiempo de vuelo cerca y lejos (video) para disparar en movimiento.
- [ ] `MechanismConstants.INTAKE_LEFT_IS_FRONT`: ¿intakeLeft es el de enfrente?
- [ ] Afinar `TurretConstants` (KP, KD, KS, KV) en Panels hasta que la torreta llegue rápido sin oscilar.
- [ ] Autónomo completo: tunear Pedro (`pedro/Tuning.java`), copiar resultados a `pedro/Constants.java`
      con `FORESIGHT_TUNED = true`, medir poses en `AutoConstants` y `PATHS_CONFIGURED = true`.
      Mientras no, el autónomo dispara las precargadas y sale de la zona.
