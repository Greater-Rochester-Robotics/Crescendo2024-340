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
                    sequence(waitSeconds(1.7).deadlineWith(Routines.prepPoop()), Routines.poop(false), Routines.intake())
                ),
                deadline(
                    swerve.followTrajectory(traj.get(1), 0.5, 1.7),
                    sequence(waitSeconds(1.3).deadlineWith(Routines.intake()), feeder.shoot().withTimeout(0.6), Routines.intake())
                ),
                deadline(
                    swerve.followTrajectory(traj.get(2), 1.8, -1.0),
                    sequence(waitSeconds(2.3).deadlineWith(Routines.intake()), feeder.shoot().withTimeout(0.6), Routines.intake())
                ),
                deadline(
                    swerve.followTrajectory(traj.get(3), 0.1, 1.1),
                    sequence(
                        waitSeconds(0.4).deadlineWith(Routines.intake()),
                        waitSeconds(0.6).deadlineWith(feeder.shoot()),
                        waitSeconds(0.5).deadlineWith(Routines.intake()),
                        feeder.shoot().withTimeout(0.6),
                        Routines.intake()
                    )
                ),
                parallel(swerve.driveSpeaker(), sequence(waitSeconds(0.5).deadlineWith(Routines.intake()), feeder.shoot()))
            )
        );
    }

    public static Command fourPieceFar(List<ChoreoTrajectory> traj) {
        return parallel(
            sequence(
                pivot.goTo(PivotConstants.BARF_FORWARD_POSITION),
                pivot.maintainPosition().withTimeout(2.25),
                pivot.targetDistance(swerve::getSpeakerDistance)
            ),
            sequence(
                deadline(
                    swerve.followTrajectory(traj.get(0), -1.0, -1.0, true),
                    sequence(waitSeconds(1.7).deadlineWith(Routines.prepPoop()), Routines.poop(false), Routines.intake())
                ),
                deadline(
                    swerve.followTrajectory(traj.get(1), 1.85, 2.5),
                    sequence(waitSeconds(2.1).deadlineWith(Routines.intake()), feeder.shoot().withTimeout(0.6), Routines.intake())
                ),
                deadline(
                    swerve.followTrajectory(traj.get(2), 0.7, 1.7),
                    sequence(waitSeconds(1.2).deadlineWith(Routines.intake()), feeder.shoot().withTimeout(0.6), Routines.intake())
                ),
                swerve.followTrajectory(traj.get(3), 1.1, -1.0).deadlineWith(Routines.intake()),
                swerve.driveSpeaker().withTimeout(0.2),
                feeder.shoot().withTimeout(0.6),
                swerve.followTrajectory(traj.get(4)).deadlineWith(Routines.intake()),
                parallel(swerve.driveSpeaker(), sequence(waitSeconds(0.6).deadlineWith(Routines.intake()), feeder.shoot()))
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
}
