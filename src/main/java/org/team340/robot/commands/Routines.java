package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.robot.Constants;

/**
 * This class is used to declare commands that require multiple subsystems.
 */
public class Routines {

    private Routines() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Deploys and runs the intake. After a note is collected, it is seated by the feeder.
     */
    public static Command intake() {
        return sequence(waitUntil(pivot::isSafeForIntake), intake.downPosition(), race(feeder.receive(), intake.intake()), feeder.seat())
            .withName("Routines.intake()");
    }

    /**
     * Intakes from the human player.
     */
    public static Command intakeHuman() {
        return parallel(
            sequence(
                deadline(
                    sequence(waitUntil(feeder::hasNote), waitUntil(() -> !feeder.hasNote())),
                    parallel(shooter.intakeHuman(), feeder.intakeHuman())
                ),
                feeder.seat()
            ),
            sequence(
                pivot.goTo(Constants.PivotConstants.INTAKE_SAFE_POSITION).unless(pivot::isSafeForIntake),
                intake.uprightPosition(),
                pivot.goTo(Constants.PivotConstants.MAX_POS)
            )
        )
            .withName("Routines.intakeHuman()");
    }

    /**
     * Intakes while ignoring note detectors.
     */
    public static Command intakeOverride() {
        return parallel(intake.intakeOverride(), feeder.shoot()).withName("Routines.intakeOverride()");
    }

    /**
     * Prepares to score in the amp by facing the amp and handing the note to the intake.
     * @param x The desired {@code x} driving speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} driving speed from {@code -1.0} to {@code 1.0}.
     */
    public static Command prepAmp(Supplier<Double> x, Supplier<Double> y) {
        return parallel(
            swerve.driveAmp(x, y),
            sequence(
                sequence(
                    parallel(
                        pivot.goTo(Constants.PivotConstants.AMP_HANDOFF_POSITION),
                        sequence(waitUntil(pivot::isSafeForIntake), intake.downPosition())
                    ),
                    sequence(
                        intake.downPosition(),
                        deadline(
                            sequence(waitUntil(() -> intake.hasNote() && !feeder.hasNote()), waitSeconds(0.1)),
                            feeder.barfForward(),
                            intake.ampHandoff()
                        )
                    )
                )
                    .unless(intake::hasNote),
                intake.ampPosition(),
                intake.maintainPosition()
            )
        );
    }

    /**
     * Prepares for a climb by raising the arms and facing the stage.
     * @param x The desired {@code x} driving speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} driving speed from {@code -1.0} to {@code 1.0}.
     */
    public static Command prepClimb(Supplier<Double> x, Supplier<Double> y) {
        return parallel(
            swerve.driveStage(x, y),
            intake.uprightPosition().onlyIf(() -> swerve.getStageDistance() >= 1.9),
            pivot.goTo(Math.toRadians(3.0))
        )
            .withName("Routines.prepClimb()");
    }

    /**
     * Barfs the note forwards out of the intake.
     */
    public static Command barfForward() {
        return sequence(
            parallel(pivot.goTo(Constants.PivotConstants.BARF_FORWARD_POSITION), intake.barfPosition()),
            parallel(feeder.barfForward(), intake.barf())
        )
            .withName("Routines.barfForward()");
    }

    /**
     * Barfs the note backwards out of the shooter.
     * @return
     */
    public static Command barfBackward() {
        return parallel(shooter.barf(), sequence(waitSeconds(0.35), parallel(feeder.barfBackward(), intake.intake())))
            .withName("Routines.barfBackward()");
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
     * Calls {@code onDisable()} for all subsystems.
     */
    public static Command onDisable() {
        return sequence(waitSeconds(6.0), parallel(climber.onDisable(), feeder.onDisable(), intake.onDisable(), pivot.onDisable()))
            .withName("Routines.onDisable()");
    }
}
