package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.team340.robot.Constants.PivotConstants;

/**
 * This class is used to declare autonomous routines.
 */
public class Autos {

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Command fivePieceAmp(List<ChoreoTrajectory> traj) {
        return parallel(
            sequence(
                pivot.goTo(PivotConstants.BARF_FORWARD_POSITION),
                pivot.maintainPosition().withTimeout(2.5),
                pivot.targetDistance(swerve::getSpeakerDistance)
            ),
            sequence(
                deadline(
                    swerve.followTrajectory(traj.get(0), true),
                    sequence(deadline(waitSeconds(1.7), Routines.prepPoop()), Routines.poop(false), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(1), 1.15, -1.0),
                    sequence(sequence(waitSeconds(1.92), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(2), 2.4, -1.0),
                    sequence(sequence(waitSeconds(2.88), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(3), 0.2, 1.1),
                    sequence(sequence(waitSeconds(0.88), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(4), 0.3, -1.0),
                    sequence(sequence(waitSeconds(0.6), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                parallel(swerve.driveSpeaker(), sequence(deadline(waitSeconds(0.25), Routines.intake()), feeder.shoot()))
            )
        );
    }

    public static Command fourPieceClose(List<ChoreoTrajectory> traj) {
        return parallel(
            pivot.targetDistance(swerve::getSpeakerDistance),
            sequence(
                deadline(swerve.followTrajectory(traj.get(0), -1.0, -1.0, true), intake.downPosition()),
                deadline(sequence(waitSeconds(1.1), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
                deadline(swerve.followTrajectory(traj.get(1)), Routines.intake()),
                Routines.intake().withTimeout(0.3),
                deadline(sequence(waitSeconds(0.7), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
                deadline(swerve.followTrajectory(traj.get(2)), Routines.intake()),
                Routines.intake().withTimeout(0.3),
                deadline(sequence(waitSeconds(0.7), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
                deadline(swerve.followTrajectory(traj.get(3)), Routines.intake()),
                Routines.intake().withTimeout(0.3),
                deadline(sequence(waitSeconds(0.7), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker())
            )
        );
    }

    public static Command fourPieceFar(List<ChoreoTrajectory> traj) {
        return parallel(
            pivot.targetDistance(swerve::getSpeakerDistance),
            sequence(
                deadline(
                    // This goes to the first shooting position.
                    swerve.followTrajectory(traj.get(0)),
                    intake.downPosition()
                ),
                swerve.driveSpeaker().withTimeout(1.0),
                feeder.shoot().withTimeout(0.1),
                deadline(
                    // this goes to pick up the first note.
                    swerve.followTrajectory(traj.get(1)),
                    Routines.intake()
                ),
                // This drives to the shooting position for the first note.
                swerve.followTrajectory(traj.get(2)),
                swerve.driveSpeaker().withTimeout(0.4),
                feeder.shoot().withTimeout(0.1),
                deadline(
                    // This drives to the second note.
                    swerve.followTrajectory(traj.get(3)),
                    Routines.intake()
                ),
                deadline(
                    // This goes to the third note.
                    swerve.followTrajectory(traj.get(4), 0.8, -1),
                    sequence(waitSeconds(1.3), feeder.shoot().withTimeout(0.4), Routines.intake())
                ),
                // This goes to the end position.
                swerve.followTrajectory(traj.get(5)),
                swerve.driveSpeaker().withTimeout(1.0),
                feeder.shoot()
            )
        );
    }
}
