package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Busca dispositivos en la configuración sin tronar si falta alguno: anota cuáles faltan para que
 * el diagnóstico los muestre y el robot siga funcionando con lo que sí tiene.
 */
public class HardwareCheck {
    private final HardwareMap hardwareMap;
    private final List<String> missing = new ArrayList<>();

    public HardwareCheck(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /** El dispositivo, o null si no está (y queda anotado con su descripción). */
    public <T> T get(Class<T> type, String name, String description) {
        T device = hardwareMap.tryGet(type, name);
        if (device == null) missing.add(name + " (" + description + ")");
        return device;
    }

    public void addMissing(String description) {
        missing.add(description);
    }

    public List<String> missing() {
        return missing;
    }

    public HardwareMap hardwareMap() {
        return hardwareMap;
    }
}
