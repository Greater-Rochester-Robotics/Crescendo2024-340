package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import org.team340.robot.RobotContainer;
import org.team340.robot.subsystems.Feeder;
import org.team340.robot.subsystems.Feeder.FeederSpeed;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Intake.IntakeState;
import org.team340.robot.subsystems.Pivot;
import org.team340.robot.subsystems.Pivot.PivotPosition;
import org.team340.robot.subsystems.Shooter;
import org.team340.robot.subsystems.Shooter.ShooterSpeed;
import org.team340.robot.subsystems.Swerve;

/**
 * This class is used to declare commands that require multiple subsystems.
 */
@Logged(strategy = Strategy.OPT_IN)
public class Routines {

    private final Feeder feeder;
    private final Intake intake;
    private final Pivot pivot;
    private final Shooter shooter;
    private final Swerve swerve;

    public Routines(RobotContainer robotContainer) {
        feeder = robotContainer.feeder;
        intake = robotContainer.intake;
        pivot = robotContainer.pivot;
        shooter = robotContainer.shooter;
        swerve = robotContainer.swerve;
    }

    /**
     * Intakes from the ground. Ends when the note is detected.
     */
    public Command intake() {
        return sequence(
            parallel(intake.apply(IntakeState.kIntake), feeder.apply(FeederSpeed.kReceive)).until(feeder::hasNote),
            new ScheduleCommand(feeder.seat())
        )
            .onlyIf(feeder::noNote)
            .withName("Routines.intake()");
    }

    /**
     * Intakes via the feeder station. Ends when the note is seated.
     */
    public Command humanLoad() {
        return sequence(
            parallel(
                feeder.apply(FeederSpeed.kBarfForward),
                pivot.apply(PivotPosition.kHumanLoad),
                shooter.apply(ShooterSpeed.kHumanLoad)
            ).until(feeder::hasNote),
            parallel(feeder.apply(FeederSpeed.kBarfForward), pivot.apply(PivotPosition.kDown)).until(feeder::noNote),
            new ScheduleCommand(feeder.seat())
        )
            .onlyIf(feeder::noNote)
            .withName("Routines.humanLoad()");
    }

    /**
     * Fixes the position of the note if it is in a deadzone.
     */
    public Command fixDeadzone() {
        return sequence(
            parallel(
                feeder.apply(FeederSpeed.kBarfForward),
                pivot.apply(PivotPosition.kFixDeadzone),
                shooter.apply(ShooterSpeed.kFixDeadzone)
            ).until(feeder::noNote),
            new ScheduleCommand(feeder.seat())
        ).withName("Routines.fixDeadzone()");
    }

    /**
     * Barfs backwards (towards the shooter). Does not end.
     */
    public Command barf() {
        return parallel(
            feeder.apply(FeederSpeed.kBarfBackward),
            intake.apply(IntakeState.kBarf),
            shooter.apply(ShooterSpeed.kBarfBackward)
        ).withName("Routines.barfBackward()");
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
     * Calls {@code onDisable()} for all subsystems.
     */
    public Command onDisable() {
        return sequence(waitSeconds(6.0), parallel(feeder.onDisable(), intake.onDisable(), pivot.onDisable())).withName(
            "Routines.onDisable()"
        );
    }
}
