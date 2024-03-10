package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.robot.Constants.PivotConstants;

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
     * Finishes the intake sequence.
     */
    public static Command finishIntake() {
        return parallel(feeder.seat(), intake.safePosition()).withName("Routines.finishIntake()");
    }

    /**
     * Intakes from the human player.
     */
    public static Command intakeHuman() {
        return parallel(
            deadline(waitUntil(feeder::hasNote).andThen(waitSeconds(0.1)), shooter.intakeHuman(), feeder.intakeHuman()),
            sequence(
                pivot.goTo(PivotConstants.INTAKE_SAFE_POSITION).unless(pivot::isSafeForIntake),
                intake.uprightPosition(),
                pivot.goTo(PivotConstants.MAX_POS)
            )
        )
            .withName("Routines.intakeHuman()");
    }

    /**
     * Finishes the human player intake sequence.
     */
    public static Command finishIntakeHuman() {
        return parallel(
            shooter.setSpeed(0).withTimeout(2.0),
            pivot.goTo(PivotConstants.DOWN_POSITION),
            sequence(waitUntil(pivot::isSafeForIntake), parallel(intake.safePosition(), sequence(feeder.reverseSeat(), feeder.seat())))
        )
            .withName("Routines.finishIntakeHuman()");
    }

    /**
     * Intakes while ignoring note detectors.
     */
    public static Command intakeOverride() {
        return parallel(intake.intakeOverride(), feeder.shoot()).withName("Routines.intakeOverride()");
    }

    /**
     * Fixes the position of the note if it is in a deadzone.
     */
    public static Command fixDeadzone() {
        return sequence(
            deadline(feeder.reverseSeat(), shooter.barfForward(), pivot.goTo(PivotConstants.FIX_DEADZONE_POSITION)),
            parallel(feeder.seat(), pivot.goTo(PivotConstants.DOWN_POSITION))
        )
            .withName("Routines.fixDeadzone()");
    }

    /**
     * Prepares to score in speaker by facing the speaker and moving the pivot.
     * @param x The desired {@code x} driving speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} driving speed from {@code -1.0} to {@code 1.0}.
     */
    public static Command prepSpeaker(Supplier<Double> x, Supplier<Double> y) {
        return parallel(swerve.driveSpeaker(x, y), pivot.targetDistance(swerve::getSpeakerDistance)).withName("Routines.prepSpeaker()");
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
                        pivot.goTo(PivotConstants.AMP_HANDOFF_POSITION),
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
            swerve.driveClimb(x, y),
            intake.uprightPosition().onlyIf(() -> swerve.getStageDistance() >= 1.9),
            pivot.goTo(PivotConstants.DOWN_POSITION)
        )
            .withName("Routines.prepClimb()");
    }

    /**
     * Prepares to feed a note to our alliance.
     * @param x The desired {@code x} driving speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} driving speed from {@code -1.0} to {@code 1.0}.
     */
    public static Command prepFeed(Supplier<Double> x, Supplier<Double> y) {
        return parallel(swerve.driveFeed(x, y), shooter.feed(swerve::pastMidline), pivot.feed(swerve::pastMidline));
    }

    /**
     * Barfs the note forwards out of the intake.
     */
    public static Command barfForward() {
        return sequence(
            parallel(pivot.goTo(PivotConstants.BARF_FORWARD_POSITION), intake.barfPosition()),
            parallel(shooter.barfForward(), feeder.barfForward(), intake.barf())
        )
            .withName("Routines.barfForward()");
    }

    /**
     * Prepares to poop the note forwards out of the intake.
     */
    public static Command prepPoop() {
        return sequence(
            intake.downPosition(),
            deadline(
                sequence(waitUntil(() -> intake.hasNote() && !feeder.hasNote()), waitSeconds(0.1)),
                feeder.barfForward(),
                intake.ampHandoff()
            ),
            intake.poopPosition()
        )
            .withName("Routines.prepPoop()");
    }

    /**
     * Poops the note out of the intake.
     * @param includePrep If {@link Routines#prepPoop()} should be called first.
     */
    public static Command poop(boolean includePrep) {
        return sequence(
            includePrep ? Routines.prepPoop() : none(),
            deadline(sequence(waitUntil(() -> !intake.hasNote()), waitSeconds(0.15)), intake.poop())
        )
            .withName("Routines.poop(" + includePrep + ")");
    }

    /**
     * Barfs the note backwards out of the shooter.
     * @return
     */
    public static Command barfBackward() {
        return parallel(shooter.barfBackward(), sequence(waitSeconds(0.35), parallel(feeder.barfBackward(), intake.intake())))
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
