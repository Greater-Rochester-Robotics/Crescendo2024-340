package org.team340.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;

/**
 * Controls the lights.
 */
public class Lights extends GRRSubsystem {

    public final DigitalOutput lights;

    /**
     * Create the lights subsystem.
     */
    public Lights() {
        super("Lights");
        lights = new DigitalOutput(RobotMap.LIGHTS);
        lights.enablePWM(0.0);
    }

    /**
     * Displays the "Has Note" signal.
     */
    public Command hasNote() {
        return runOnce(() -> {});
    }
}
