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
        return parallel(
            pivot.goTo(PivotConstants.INTAKE_SAFE_POSITION_CLEARED).unless(pivot::isSafeForIntake),
            sequence(waitUntil(pivot::isSafeForIntake), intake.downPosition(), race(feeder.receive(), intake.intake()), feeder.seat())
        )
            .withName("Routines.intake()");
    }

    /**
     * Finishes the intake sequence.
     */
    public static Command finishIntake() {
        return parallel(feeder.seat(), sequence(intake.safePosition())).withName("Routines.finishIntake()");
    }

    /**
     * Intakes from the human player.
     */
    public static Command intakeHuman() {
        return sequence(
            pivot.goTo(PivotConstants.INTAKE_SAFE_POSITION_CLEARED).unless(pivot::isSafeForIntake),
            intake.uprightPosition(),
            pivot.goTo(PivotConstants.MAX_POS),
            deadline(waitUntil(feeder::hasNote).andThen(waitSeconds(0.1)), shooter.intakeHuman(), feeder.intakeHuman())
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
     * Barfs the note forwards out of the intake.
     */
    public static Command barfForward() {
        return sequence(
            parallel(pivot.goTo(PivotConstants.BARF_FORWARD_POSITION), intake.barfPosition()).withTimeout(0.5),
            parallel(pivot.goTo(PivotConstants.BARF_FORWARD_POSITION), feeder.barfForward(), intake.barf())
        )
            .withName("Routines.barfForward()");
    }

    /**
     * Barfs the note backwards out of the shooter.
     */
    public static Command barfBackward() {
        return parallel(
            shooter.barfBackward(),
            sequence(
                parallel(waitSeconds(0.35), pivot.goTo(PivotConstants.DOWN_POSITION), intake.downPosition()),
                parallel(feeder.barfBackward(), intake.intake())
            )
        )
            .withName("Routines.barfBackward()");
    }

    /**
     * Prepares to poop the note forwards out of the intake.
     */
    public static Command prepPoop() {
        return sequence(handoff(), intake.poopPosition()).withName("Routines.prepPoop()");
    }

    /**
     * Poops the note out of the intake.
     */
    public static Command poop() {
        return deadline(sequence(waitUntil(() -> !intake.hasNote()), waitSeconds(0.15)), intake.poop()).withName("Routines.poop()");
    }

    /**
     * Returns the note from the feeder back to the intake.
     */
    public static Command handoff() {
        return sequence(
            parallel(pivot.goTo(PivotConstants.HANDOFF_POSITION), sequence(waitUntil(pivot::isSafeForIntake), intake.handoffPosition())),
            deadline(
                sequence(waitUntil(() -> intake.hasNote() && !feeder.hasNote()), waitSeconds(0.1)),
                feeder.barfForward(),
                intake.handoff()
            )
        );
    }

    /**
     * Feeds notes directly from the intake straight into the shooter.
     * @param rampSpeed The speed to ramp the shooter by in percent duty cycle / second.
     */
    public static Command feedThrough(Supplier<Double> rampSpeed) {
        return parallel(intake.intake(), feeder.shoot(true), shooter.driveManual(rampSpeed)).withName("Routines.feedThrough()");
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
     * Calls {@code onDisable()} for all subsystems.
     */
    public static Command onDisable() {
        return sequence(waitSeconds(6.0), parallel(feeder.onDisable(), intake.onDisable(), pivot.onDisable()))
            .withName("Routines.onDisable()");
    }
}
