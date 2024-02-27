package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

/**
 * This class is used to declare autonomous routines.
 */
public class Autos {

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * An example auto.
     */
    public static Command example() {
        return sequence(swerve.drive(() -> 0.1, () -> 0.0, () -> 0.0, true).withTimeout(1.0));
    }

    public static Command fourPieceLeft(List<ChoreoTrajectory> traj) {
        return parallel(
            pivot.targetDistance(swerve::getSpeakerDistance),
            sequence(
                deadline(swerve.followTrajectory(traj.get(0), -1.0, true), intake.downPosition()),
                deadline(sequence(waitSeconds(0.65), feeder.shoot().withTimeout(0.6)), swerve.driveSpeaker()),
                deadline(swerve.followTrajectory(traj.get(1)), Routines.intake()),
                deadline(sequence(swerve.followTrajectory(traj.get(2)), waitSeconds(0.2)), Routines.intake().withTimeout(0.3)),
                feeder.shoot().withTimeout(0.6),
                deadline(swerve.followTrajectory(traj.get(3)), Routines.intake()),
                deadline(swerve.followTrajectory(traj.get(4)), Routines.intake().withTimeout(0.3)),
                feeder.shoot().withTimeout(0.6),
                parallel(swerve.followTrajectory(traj.get(5)), Routines.intake()),
                deadline(sequence(waitSeconds(1.0), feeder.shoot().withTimeout(0.6)), swerve.driveSpeaker())
            )
        );
    }

    public static Command fourPieceFront(List<ChoreoTrajectory> traj) {
        return parallel(
            pivot.targetDistance(swerve::getSpeakerDistance),
            sequence(
                deadline(swerve.followTrajectory(traj.get(0), 1.0, true), intake.downPosition()),
                deadline(sequence(waitSeconds(1.2), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
                deadline(sequence(swerve.followTrajectory(traj.get(1)), waitSeconds(0.5)), Routines.intake()),
                swerve.followTrajectory(traj.get(2), 0.7),
                deadline(sequence(waitSeconds(0.6), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
                deadline(sequence(swerve.followTrajectory(traj.get(3), 1.4), waitSeconds(0.5)), Routines.intake()),
                deadline(sequence(waitSeconds(0.6), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
                deadline(sequence(swerve.followTrajectory(traj.get(4), 1.2), waitSeconds(0.5)), Routines.intake()),
                deadline(sequence(waitSeconds(0.6), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker())
            )
        );
    }

    public static Command fourPieceRight(List<ChoreoTrajectory> traj) {
        return parallel(
            pivot.targetDistance(swerve::getSpeakerDistance),
            sequence(
                deadline(swerve.followTrajectory(traj.get(0), -1.0, true), intake.downPosition()),
                deadline(sequence(waitSeconds(1.1), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
                deadline(swerve.followTrajectory(traj.get(1)), Routines.intake()),
                deadline(swerve.followTrajectory(traj.get(2)), Routines.intake().withTimeout(0.3)),
                deadline(sequence(waitSeconds(0.25), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
                deadline(swerve.followTrajectory(traj.get(3)), Routines.intake()),
                deadline(swerve.followTrajectory(traj.get(4)), Routines.intake().withTimeout(0.3)),
                deadline(sequence(waitSeconds(0.25), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
                deadline(swerve.followTrajectory(traj.get(5)), Routines.intake()),
                deadline(swerve.followTrajectory(traj.get(6)), Routines.intake().withTimeout(0.3)),
                deadline(sequence(waitSeconds(0.25), feeder.shoot().withTimeout(0.6)), swerve.driveSpeaker())
            )
        );
    }
}
