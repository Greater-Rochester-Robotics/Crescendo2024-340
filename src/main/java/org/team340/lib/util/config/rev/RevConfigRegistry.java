package org.team340.lib.util.config.rev;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import java.text.Collator;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.TreeSet;

/**
 * Utilities for REV hardware configs.
 */
public final class RevConfigRegistry {

    public static final double EPSILON = 1e-4;
    public static final double BURN_FLASH_SLEEP = 100.0;
    public static final double CHECK_SLEEP = 25.0;
    public static final int DEFAULT_SET_ITERATIONS = 3;
    public static final double PERIODIC_INTERVAL = 100.0;

    private static final Collection<String> errors = new TreeSet<>(ErrorComparator.getInstance());
    private static final List<Runnable> periodicCallbacks = new ArrayList<>();

    private RevConfigRegistry() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Blocks the thread to ensure a safe burn to flash.
     */
    static void burnFlashSleep() {
        if (!RobotBase.isSimulation()) {
            try {
                Thread.sleep((long) BURN_FLASH_SLEEP);
            } catch (Exception e) {}
        }
    }

    /**
     * Saves a configuration error string to be logged.
     * @param errorString The error string.
     */
    static void addError(String errorString) {
        errors.add(errorString);
    }

    /**
     * Saves a callback to a runnable for setting the periodic frame period.
     * @param callback The runnable.
     */
    static void addPeriodic(Runnable callback) {
        periodicCallbacks.add(callback);
    }

    /**
     * Prints unsuccessful configurations to stdout.
     * Useful for debugging, should be ran after initializing all hardware.
     */
    public static void printError() {
        if (errors.size() <= 0) {
            System.out.println("\nAll REV hardware configured successfully\n");
        } else {
            DriverStation.reportWarning("\nErrors while configuring " + errors.size() + " options on REV hardware:", false);

            for (String errorString : errors) {
                DriverStation.reportWarning("\t" + errorString, false);
            }
            DriverStation.reportWarning("\n", false);
            errors.clear();
        }
    }

    /**
     * Initializes periodically applying frame periods from {@code periodicCallbacks} list.
     * @param robot Robot to call {@link TimedRobot#addPeriodic(Runnable, double) addPeriodic()} on.
     */
    public static void init(TimedRobot robot) {
        robot.addPeriodic(
            () -> {
                for (Runnable callback : periodicCallbacks) {
                    callback.run();
                }
            },
            PERIODIC_INTERVAL
        );
    }

    private static final class ErrorComparator implements Comparator<String> {

        private static ErrorComparator instance;
        private final Collator localeComparator = Collator.getInstance();

        public static ErrorComparator getInstance() {
            if (instance == null) instance = new ErrorComparator();
            return instance;
        }

        private ErrorComparator() {}

        @Override
        public int compare(String arg0, String arg1) {
            int idDiff;
            try {
                idDiff = Integer.parseInt(arg0.split(" ")[1]) - Integer.parseInt(arg1.split(" ")[1]);
            } catch (Exception e) {
                idDiff = 0;
            }

            if (idDiff == 0) {
                int localeDiff = localeComparator.compare(arg0, arg1);
                if (localeDiff == 0) return 1; else return localeDiff;
            } else {
                return idDiff;
            }
        }
    }
}