package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Feeder;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Swerve;

/**
 * The Routines class contains command compositions that require
 * multiple subsystems, such as sequences or parallel command groups.
 */
public final class Routines {

    private final Feeder feeder;
    private final Intake intake;
    private final Swerve swerve;

    public Routines(Robot robot) {
        feeder = robot.feeder;
        intake = robot.intake;
        swerve = robot.swerve;
    }

    public Command intake() {
        return parallel(feeder.pickup(), intake.intake());
    }

    /**
     * An example routine.
     */
    public Command example() {
        return sequence(
            swerve.drive(() -> 0.5, () -> 0.0, () -> 0.0).withTimeout(1.0),
            swerve.stop(false).withTimeout(1.0),
            print("Done!")
        ).withName("Routines.example()");
    }
}
