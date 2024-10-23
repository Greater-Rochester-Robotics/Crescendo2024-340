package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import org.team340.lib.dashboard.GRRDashboard;
import org.team340.lib.util.Alliance;
import org.team340.robot.RobotContainer;
import org.team340.robot.subsystems.Feeder;
import org.team340.robot.subsystems.Feeder.FeederSpeed;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Intake.IntakeState;
import org.team340.robot.subsystems.Pivot;
import org.team340.robot.subsystems.Pivot.PivotPosition;
import org.team340.robot.subsystems.Shooter;
import org.team340.robot.subsystems.Swerve;

@Logged(strategy = Strategy.OPT_IN)
public class Autos {

    private final Feeder feeder;
    private final Intake intake;
    private final Pivot pivot;
    private final Shooter shooter;
    private final Swerve swerve;
    private final Routines routines;

    public Autos(RobotContainer robotContainer) {
        feeder = robotContainer.feeder;
        intake = robotContainer.intake;
        pivot = robotContainer.pivot;
        shooter = robotContainer.shooter;
        swerve = robotContainer.swerve;
        routines = robotContainer.routines;

        preloadDumpC1C2();
    }

    public void preloadDumpC1C2() {
        String name = "preloadDumpC1C2";
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);

        if (traj.isPresent()) {
            Command command = sequence(
                deadline(
                    sequence(
                        swerve.resetPose(() -> traj.get().getInitialPose(Alliance.isRed())),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 0),
                            sequence(intake.apply(IntakeState.kRetract).withTimeout(0.5), routines.intake())
                        ),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 1),
                            sequence(waitSeconds(1.0), swerve.trajAimSpeaker()),
                            sequence(routines.intake().withTimeout(0.5), intake.apply(IntakeState.kRetract))
                        ),
                        parallel(
                            intake.apply(IntakeState.kRetract),
                            feeder.apply(FeederSpeed.kShoot),
                            swerve.driveSpeaker(() -> 0.0, () -> 0.0)
                        ).withTimeout(0.3),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 2),
                            sequence(intake.apply(IntakeState.kRetract).withTimeout(0.5), routines.intake())
                        ),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 3),
                            sequence(waitSeconds(1.2), swerve.trajAimSpeaker()),
                            sequence(routines.intake().withTimeout(0.5), intake.apply(IntakeState.kRetract))
                        ),
                        parallel(
                            intake.apply(IntakeState.kRetract),
                            feeder.apply(FeederSpeed.kShoot),
                            swerve.driveSpeaker(() -> 0.0, () -> 0.0)
                        ).withTimeout(0.3),
                        deadline(swerve.autoFactory().trajectoryCommand(name, 4), routines.intake()),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 5),
                            sequence(waitSeconds(0.5), swerve.trajAimSpeaker()),
                            sequence(routines.intake().withTimeout(0.5), intake.apply(IntakeState.kRetract))
                        ),
                        parallel(
                            intake.apply(IntakeState.kRetract),
                            feeder.apply(FeederSpeed.kShoot),
                            swerve.driveSpeaker(() -> 0.0, () -> 0.0)
                        ).withTimeout(0.3)
                    ),
                    shooter.targetSpeaker(swerve::getSpeakerDistance),
                    pivot.targetSpeaker(swerve::getSpeakerDistance)
                ),
                parallel(
                    intake.apply(IntakeState.kRetract),
                    pivot.apply(PivotPosition.kDown),
                    sequence(swerve.autoFactory().trajectoryCommand(name, 6), swerve.lockWheels())
                )
            );
            GRRDashboard.addAuto("Pre-load Dump 1 + 2", command, traj.get());
        }
    }
}
