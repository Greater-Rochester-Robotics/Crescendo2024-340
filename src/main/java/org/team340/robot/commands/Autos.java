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

    public static Command fourPiece(List<ChoreoTrajectory> traj) {
        return parallel(
            Routines.prepShootSpeaker(swerve::getSpeakerDistance),
            sequence(
                deadline(swerve.followTrajectory(traj.get(0), true), intake.intakeDown()),
                waitSeconds(1.0),
                feeder.shootNote(),
                parallel(
                    intake.intake(),
                    sequence(
                        swerve.followTrajectory(traj.get(1)),
                        feeder.shootNote().withTimeout(0.75),
                        swerve.followTrajectory(traj.get(2)),
                        feeder.shootNote().withTimeout(0.75),
                        swerve.followTrajectory(traj.get(3)),
                        feeder.shootNote().withTimeout(0.75)
                    )
                )
            )
        );
    }

    public static Command fourPieceRight(List<ChoreoTrajectory> traj) {
        return parallel(
            Routines.prepShootSpeaker(swerve::getSpeakerDistance),
            sequence(
                deadline(swerve.followTrajectory(traj.get(0), 0.5, true), intake.intakeDown()),
                deadline(sequence(waitSeconds(1.0), feeder.shootNote().withTimeout(0.75)), swerve.driveSpeaker()),
                feeder.shootNote(),
                parallel(
                    intake.intake(),
                    sequence(
                        swerve.followTrajectory(traj.get(1)),
                        waitSeconds(0.5),
                        swerve.followTrajectory(traj.get(2), 1.5),
                        deadline(feeder.shootNote().withTimeout(0.75), swerve.driveSpeaker()),
                        swerve.followTrajectory(traj.get(3)),
                        waitSeconds(0.5),
                        swerve.followTrajectory(traj.get(4), 1.5),
                        deadline(feeder.shootNote().withTimeout(0.75), swerve.driveSpeaker())
                    )
                )
            )
        );
    }
}
