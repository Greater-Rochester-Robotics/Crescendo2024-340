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

    public Autos(RobotContainer robotContainer) {
        feeder = robotContainer.feeder;
        intake = robotContainer.intake;
        pivot = robotContainer.pivot;
        shooter = robotContainer.shooter;
        swerve = robotContainer.swerve;

        preloadDumpC1C2();
        preloadDumpC2C1();
    }

    /**
     * Intakes a note.
     * @param delay The delay to deploy the intake in seconds.
     * @param seat If the note should be seated.
     */
    private Command intake(double delay, boolean seat) {
        return sequence(
            intake.apply(IntakeState.kRetract).withTimeout(delay),
            parallel(intake.apply(IntakeState.kIntake), feeder.apply(FeederSpeed.kReceive))
                .onlyIf(feeder::noNote)
                .until(feeder::hasNote),
            parallel(seat ? feeder.seat() : none(), intake.apply(IntakeState.kRetract))
        ).withName("Autos.intake(" + delay + ", " + seat + ")");
    }

    /**
     * Aims the drive at the speaker and shoots a note.
     * The pivot and shooter wheels are expected to be\[]
     * already running and targeting the speaker.
     * @param timeout The timeout in seconds to shoot.
     */
    private Command shoot(double delay, double timeout) {
        return parallel(
            intake.apply(IntakeState.kRetract),
            sequence(waitSeconds(delay), feeder.apply(FeederSpeed.kShoot)),
            swerve.driveSpeaker(() -> 0.0, () -> 0.0)
        )
            .withTimeout(timeout)
            .withName("Autos.shoot(" + timeout + ")");
    }

    public void preloadDumpC1C2() {
        String name = "preloadDumpC1C2";
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);

        if (traj.isPresent()) {
            Command command = sequence(
                deadline(
                    sequence(
                        swerve.resetPose(() -> traj.get().getInitialPose(Alliance.isRed())),
                        deadline(swerve.autoFactory().trajectoryCommand(name, 0), intake(0.5, false)),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 1),
                            sequence(waitSeconds(1.0), swerve.trajAimSpeaker()),
                            intake(0.0, true)
                        ),
                        shoot(0.35, 0.6),
                        deadline(swerve.autoFactory().trajectoryCommand(name, 2), intake(0.5, false)),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 3),
                            sequence(waitSeconds(1.2), swerve.trajAimSpeaker()),
                            intake(0.0, true)
                        ),
                        shoot(0.35, 0.6),
                        deadline(swerve.autoFactory().trajectoryCommand(name, 4), intake(0.0, false)),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 5),
                            sequence(waitSeconds(0.5), swerve.trajAimSpeaker()),
                            intake(0.0, true)
                        ),
                        shoot(0.35, 0.6)
                    ),
                    shooter.targetSpeaker(swerve::getSpeakerDistance),
                    pivot.targetSpeaker(swerve::getSpeakerDistance)
                ),
                parallel(
                    sequence(swerve.autoFactory().trajectoryCommand(name, 6), swerve.lockWheels()),
                    intake.apply(IntakeState.kRetract),
                    pivot.apply(PivotPosition.kDown)
                )
            );
            GRRDashboard.addAuto("Pre-load Dump 1 + 2", command, traj.get());
        }
    }

    public void preloadDumpC2C1() {
        String name = "preloadDumpC2C1";
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);

        if (traj.isPresent()) {
            Command command = sequence(
                deadline(
                    sequence(
                        swerve.resetPose(() -> traj.get().getInitialPose(Alliance.isRed())),
                        deadline(swerve.autoFactory().trajectoryCommand(name, 0), intake(0.5, false)),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 1),
                            sequence(waitSeconds(1.0), swerve.trajAimSpeaker()),
                            intake(0.0, true)
                        ),
                        shoot(0.35, 0.6),
                        deadline(swerve.autoFactory().trajectoryCommand(name, 2), intake(0.5, false)),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 3),
                            sequence(waitSeconds(1.2), swerve.trajAimSpeaker()),
                            intake(0.0, true)
                        ),
                        shoot(0.35, 0.6),
                        deadline(swerve.autoFactory().trajectoryCommand(name, 4), intake(0.0, false)),
                        deadline(
                            swerve.autoFactory().trajectoryCommand(name, 5),
                            sequence(waitSeconds(0.5), swerve.trajAimSpeaker()),
                            intake(0.0, true)
                        ),
                        shoot(0.35, 0.6)
                    ),
                    shooter.targetSpeaker(swerve::getSpeakerDistance),
                    pivot.targetSpeaker(swerve::getSpeakerDistance)
                ),
                parallel(
                    sequence(swerve.autoFactory().trajectoryCommand(name, 6), swerve.lockWheels()),
                    intake.apply(IntakeState.kRetract),
                    pivot.apply(PivotPosition.kDown)
                )
            );
            GRRDashboard.addAuto("Pre-load Dump 2 + 1", command, traj.get());
        }
    }
}
