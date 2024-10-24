package org.team340.lib.util;

public final class StringUtil {

    private StringUtil() {
        throw new AssertionError("This is a utility class!");
    }

    public static String formatRadians(double radians) {
        return String.format("%.2f", Math.toDegrees(radians));
    }
}
