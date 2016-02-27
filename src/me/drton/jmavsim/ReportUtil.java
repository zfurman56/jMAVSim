package me.drton.jmavsim;

import javax.vecmath.Vector3d;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;

/**
 * A class containing helper methods for producing simulation reports.
 */
public final class ReportUtil {
    private static final DecimalFormat df = new DecimalFormat("0.#####", DecimalFormatSymbols.getInstance(Locale.ENGLISH));

    private ReportUtil() {
    }

    /**
     * Converts a vector to a shorter string.
     * When numbers are changing quickly on the screen, there may not be time to read the E notation.
     */
    public static String toShortString(Vector3d vec) {
        return String.format("[%s; %s; %s]",
                df.format(vec.getX()),
                df.format(vec.getY()),
                df.format(vec.getZ()));
    }
}
