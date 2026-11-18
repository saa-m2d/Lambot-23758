package org.firstinspires.ftc.teamcode.util;

/**
 * Alianza y las CELLS de su HIVE. Cada CELL tiene 4 AprilTags (manual de BIOBUZZ, sección 9.9):
 *   roja:  30-33 lado opuesto al público, 34-37 lado del público
 *   azul:  42-45 lado opuesto al público, 38-41 lado del público
 */
public enum Alliance {
    RED("ROJA", 30, 34),
    BLUE("AZUL", 42, 38);

    /** Las dos CELLS del HIVE de una alianza. */
    public enum Cell {
        FAR("lado opuesto al público"),
        AUDIENCE("lado del público");

        public final String label;
        Cell(String label) { this.label = label; }

        public Cell other() { return this == FAR ? AUDIENCE : FAR; }
    }

    public final String label;
    private final int farFirstTag;
    private final int audienceFirstTag;

    Alliance(String label, int farFirstTag, int audienceFirstTag) {
        this.label = label;
        this.farFirstTag = farFirstTag;
        this.audienceFirstTag = audienceFirstTag;
    }

    /** La CELL de esta alianza a la que pertenece un tag, o null si el tag no es de esta alianza. */
    public Cell cellOf(int tagId) {
        if (tagId >= farFirstTag && tagId < farFirstTag + 4) return Cell.FAR;
        if (tagId >= audienceFirstTag && tagId < audienceFirstTag + 4) return Cell.AUDIENCE;
        return null;
    }

    public Alliance other() { return this == RED ? BLUE : RED; }
}
