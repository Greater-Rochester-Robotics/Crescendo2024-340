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
                pivot.maintainPosition().withTimeout(2.0),
                pivot.targetDistance(swerve::getSpeakerDistance)
            ),
            sequence(
                deadline(
                    swerve.followTrajectory(traj.get(0), true),
                    sequence(
                        sequence(
                            deadline(waitSeconds(1.25), intake.safePosition()),
                            parallel(feeder.barfForward(), intake.barf()).withTimeout(0.25)
                        ),
                        Routines.intake()
                    )
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(1), 0.9),
                    sequence(sequence(waitSeconds(1.7), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(2), 1.65),
                    sequence(sequence(waitSeconds(2.55), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(3), 0.1),
                    sequence(sequence(waitSeconds(1.15), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(4), 0.05),
                    sequence(sequence(waitSeconds(0.15), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                waitSeconds(0.25),
                feeder.shoot()
            )
        );
    }

    public static Command fourPieceClose(List<ChoreoTrajectory> traj) {
        return parallel(
            pivot.targetDistance(swerve::getSpeakerDistance),
            sequence(
                deadline(swerve.followTrajectory(traj.get(0), -1.0, true), intake.downPosition()),
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
                deadline(swerve.followTrajectory(traj.get(0), -1.0, true), intake.downPosition()),
                deadline(sequence(waitSeconds(1.15), feeder.shoot().withTimeout(0.75)), swerve.driveSpeaker()),
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
