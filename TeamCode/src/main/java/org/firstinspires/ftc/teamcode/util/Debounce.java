package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Una condición que solo cuenta si se cumple sin interrupción durante cierto tiempo. */
public class Debounce {
    private final ElapsedTime timer = new ElapsedTime();
    private boolean previous;

    /** @return true si la condición lleva al menos {@code seconds} cumpliéndose seguido. */
    public boolean update(boolean condition, double seconds) {
        if (!condition || !previous) timer.reset();
        previous = condition;
        return condition && timer.seconds() >= seconds;
    }
}
