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

### Servos (tipo: Servo, para todos, durante la Fase 1)

| Hub | Puerto | Nombre | Servo |
|---|---|---|---|
| Control Hub | 4 | `turretLeft` | Axon MINI MK2 |
| Control Hub | 5 | `flowerLeft` | Swyft speed servo |
| Expansion Hub | 0 | `tope` | Axon MINI MK2 |
| Expansion Hub | 1 | `turretRight` | Axon MINI MK2 |
| Expansion Hub | 2 | `flowerRight` | Swyft speed servo |

- [ ] Los 5 servos están como tipo **Servo**, aunque alguno resulte ser continuo. La prueba 2 lo
      descubre, y después se cambia el tipo de los que sean continuos.

### Sensores

| Dónde | Nombre | Tipo |
|---|---|---|
| Expansion Hub, I2C bus 0 | `pinpoint` | goBILDA Pinpoint Odometry Computer |
| Control Hub, USB 3.0 (azul) | `limelight` | Limelight 3A (aparece con Scan) |

- [ ] Pinpoint configurado. Su dirección I2C es fija (0x31); no hay nada que cambiar.
- [ ] Limelight conectada con cable USB-A a USB-C al puerto **azul**. No usar el USB 2.0.

## 2. Servos: modos y PWM

No hay programador Axon, así que cada servo trabaja en el modo que ya trae.

- El código manda pulsos de **500 a 2500 µs** (posición 0.0 a 1.0). Es el rango del Axon MK2 en
  modo posición; de fábrica recorre unos 350° en ese rango (0.175° por µs).
- Si un Axon resultara estar en modo continuo, su rango útil es 1050–1950 µs y 0.5 es "detenido".
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

## 5. Home de la torreta (procedimiento provisional)

Home = la torreta apuntando a la **izquierda** del robot (90° antihorario desde el frente).

Los dos Axon mueven el mismo eje. Si cada uno recibe una posición distinta, pelean entre sí y se
dañan. Por eso primero se encuentra la relación entre ambos, con la prueba 2:

1. Con todo suelto, lleva la torreta a home a mano y pon una marca visible (cinta o plumón).
2. Elige `turretLeft`, energízalo (A) y ajusta hasta que la torreta quede exacto en la marca.
   Guarda con Start. Suéltalo (A).
3. Elige `turretRight`, energízalo y ajusta hasta la misma marca. Guarda con Start. Suéltalo.
4. Repite 2 y 3 en un segundo punto, unos 45° lejos de home.
5. Compara los dos puntos: si al subir la posición de un servo la del otro baja, están **en espejo**;
   si suben juntos, van en el **mismo sentido**. Ajústalo con el botón del stick izquierdo.
6. Deja cada servo en su posición de home (sueltos) y presiona X para **unirlos**.
7. Energiza (A): ahora se mueven juntos. Recorre todo el rango despacio y anota los límites.

Si al paso 2 el servo gira sin parar, está en modo continuo: regresa a 0.5, suéltalo y avisa,
porque el plan de la torreta cambia.

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
| turretLeft | | home: ____ segundo punto: ____ |
| turretRight | | home: ____ segundo punto: ____ |
| torreta unida | espejo / mismo sentido | offset: ____ límite izq: ____ límite der: ____ |
| tope | | contiene: ____ libera: ____ |
| flowerLeft | | guardado: ____ extrayendo: ____ |
| flowerRight | | guardado: ____ extrayendo: ____ |

- Grados reales de torreta entre los dos límites (medidos con transportador): ____

**Pinpoint**

- Estado READY: sí / no
- Dirección pod X: FORWARD / REVERSED. Dirección pod Y: FORWARD / REVERSED
- Tras 10 vueltas sobre su centro, X y Y regresaron a: ____ , ____ cm

**Limelight**

- Conexión y fps: ____
- Tags que ve desde la posición de inicio: ____
- ¿MegaTag 2 se mantiene estable al girar el chasis?: sí / no
- Altura de la lente sobre el piso: ____ cm
