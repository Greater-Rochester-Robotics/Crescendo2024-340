package org.team340.lib.dashboard;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedHashMap;
import java.util.Map;
import org.team340.lib.util.Alliance;

/**
 * Utility class for interfacing with GRRDashboard.
 */
public final class GRRDashboard {

    private GRRDashboard() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("GRRDashboard");
    private static final EventLoop periodic = new EventLoop();

    private static final BooleanPublisher robotBlueAlliance = nt.getBooleanTopic("Robot/blueAlliance").publish();
    private static final BooleanPublisher robotEnabled = nt.getBooleanTopic("Robot/enabled").publish();
    private static final DoublePublisher robotMatchTime = nt.getDoubleTopic("Robot/matchTime").publish();
    private static final DoublePublisher robotVoltage = nt.getDoubleTopic("Robot/voltage").publish();

    private static final BooleanSubscriber allianceOverrideActiveSub = nt
        .getBooleanTopic("AllianceOverride/active")
        .subscribe(false);
    private static final BooleanSubscriber allianceOverrideIsBlueSub = nt
        .getBooleanTopic("AllianceOverride/isBlue")
        .subscribe(false);

    private static final Map<String, Pair<Command, String>> autoOptions = new LinkedHashMap<>(); // { id: [command, json] }
    private static final StringPublisher activeAutoPub = nt.getStringTopic("Autos/active").publish();
    private static final StringSubscriber selectedAutoSub;

    private static Command selectedAuto = Commands.none();

    static {
        String defaultAuto = "";
        selectedAutoSub = nt.getStringTopic("Autos/selected").subscribe(defaultAuto);
        activeAutoPub.set(defaultAuto);
    }

    /**
     * Gets the command of the selected auto.
     */
    public static Command getSelectedAuto() {
        return selectedAuto;
    }

    /**
     * Syncs data with the dashboard. Must be called
     * periodically in order for this class to function.
     */
    public static void update() {
        robotBlueAlliance.set(Alliance.isBlue());
        robotEnabled.set(DriverStation.isEnabled());
        robotMatchTime.set(DriverStation.getMatchTime());
        robotVoltage.set(RobotController.getBatteryVoltage());

        if (allianceOverrideActiveSub.get()) {
            Alliance.enableOverride(allianceOverrideIsBlueSub.get());
        } else {
            Alliance.disableOverride();
        }

        for (String id : selectedAutoSub.readQueueValues()) {
            var entry = autoOptions.get(id);
            if (entry != null) {
                activeAutoPub.set(id);
                selectedAuto = entry.getFirst();
            }
        }

        periodic.poll();
    }

    /**
     * Binds an action to the dashboard's update loop.
     * @param action The action to bind.
     */
    static void bind(Runnable action) {
        periodic.bind(action);
    }
}
