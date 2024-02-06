package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

// TODO Discuss bindings and what commands are needed

/**
 * This class is used to declare commands that require multiple subsystems.
 */
public class Routines {

    private Routines() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * An example routine.
     */
    public static Command example() {
        return sequence(swerve.drive(() -> 0.1, () -> 0.0, () -> 0.0, true).withTimeout(1.0));
    }

    /**
     * This command takes the current position, uses it to decide the angle to shoot at, then shoots the note.
     * Note that this will not correct the robot's yaw or position.
     * @param robotPosition This is the position used for the math, note that it must be a supplier.
     * @return This command.
     */
    public Command shootSpeaker(Supplier<Pose2d> robotPosition) {
        return parallel(
            prepShootSpeaker(robotPosition),
            sequence(waitUntil(() -> shooter.hasReachedSpeed() && pivot.isOnTarget()), feeder.shootNote())
        );
    }

    public Command prepShootSpeaker(Supplier<Pose2d> robotPosition) {
        //TODO: this needs actual not zeros.
        return parallel(shooter.setSpeed(0.0), pivot.goToAngle(0.0));
    }

    public Command noteBackToIntake() {
        return deadline(
            sequence(
                // waitUntil(() -> intake.getNoteDetector() && !feeder.getNoteDetector()),
                waitSeconds(0.0001)
            ),
            shooterFeederSpit()
        );
    }

    /**
     * Designed to fully barf and designed for handoff back to intake.
     * @return This command.
     */
    public Command shooterFeederSpit() {
        return parallel(shooter.spit(), feeder.spit());
    }
}
