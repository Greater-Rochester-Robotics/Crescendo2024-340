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
                    sequence(
                        deadline(
                            waitSeconds(1.7),
                            sequence(
                                intake.downPosition(),
                                deadline(waitUntil(() -> intake.hasNote() && !feeder.hasNote()), feeder.barfForward(), intake.ampHandoff()),
                                intake.poopPosition()
                            )
                        ),
                        intake.poop().withTimeout(0.5),
                        Routines.intake()
                    )
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(1), 1.15),
                    sequence(sequence(waitSeconds(1.85), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(2), 2.4),
                    sequence(sequence(waitSeconds(2.95), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(3), 0.2),
                    sequence(sequence(waitSeconds(1.2), feeder.shoot().withTimeout(0.6)), Routines.intake())
                ),
                Routines.intake().withTimeout(0.3),
                deadline(
                    swerve.followTrajectory(traj.get(4), 0.1),
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
