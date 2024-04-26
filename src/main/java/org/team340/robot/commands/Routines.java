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
     * Safely moves the intake arm by moving the pivot if necessary.
     * @param intakeCommand A command for the intake.
     */
    public static Command safeArm(Command intakeCommand) {
        return parallel(
            pivot.goTo(PivotConstants.INTAKE_SAFE_POSITION_CLEARED).unless(pivot::isSafeForIntake),
            sequence(deadline(waitUntil(pivot::isSafeForIntake), intake.maintainPosition()), intakeCommand)
        )
            .withName("Routines.safeArm(" + intakeCommand.getName() + ")");
    }

    /**
     * Deploys and runs the intake. After a note is collected, it is seated by the feeder.
     */
    public static Command intake() {
        return parallel(
            sequence(
                safeArm(intake.downPosition()),
                race(feeder.receive(), intake.intake()),
                deadline(feeder.seat(), intake.maintainPosition())
            ),
            lights.intaking()
        )
            .withName("Routines.intake()");
    }

    /**
     * Finishes the intake sequence.
     */
    public static Command finishIntake() {
        return race(feeder.seat(), sequence(safeArm(intake.safePosition()), intake.maintainPosition())).withName("Routines.finishIntake()");
    }

    /**
     * Intakes from the human player.
     */
    public static Command intakeHuman() {
        return parallel(
            sequence(
                safeArm(intake.uprightPosition()),
                deadline(pivot.goTo(PivotConstants.MAX_POS), intake.maintainPosition()),
                deadline(
                    waitUntil(feeder::hasNote).andThen(waitSeconds(0.1)),
                    shooter.intakeHuman(),
                    feeder.intakeHuman(),
                    intake.maintainPosition()
                )
            ),
            lights.intaking()
        )
            .withName("Routines.intakeHuman()");
    }

    /**
     * Finishes the human player intake sequence.
     */
    public static Command finishIntakeHuman() {
        return parallel(
            shooter.setSpeed(0.0).withTimeout(2.0),
            pivot.goTo(PivotConstants.DOWN_POSITION),
            sequence(
                deadline(waitUntil(pivot::isSafeForIntake), intake.maintainPosition()),
                parallel(intake.safePosition(), sequence(feeder.reverseSeat(), feeder.seat()))
            )
        )
            .withName("Routines.finishIntakeHuman()");
    }

    /**
     * Barfs the note forwards out of the intake.
     */
    public static Command barf() {
        return parallel(
            sequence(pivot.goTo(PivotConstants.BARF_POSITION), pivot.maintainPosition()),
            sequence(
                deadline(waitUntil(pivot::isSafeForIntake), intake.maintainPosition()),
                intake.barfPosition(),
                parallel(shooter.barf(), feeder.barf(), intake.barf())
            ),
            lights.flames()
        )
            .withName("Routines.barf()");
    }

    /**
     * Prepares to poop the note forwards out of the intake.
     */
    public static Command prepPoop() {
        return parallel(sequence(handoff(), intake.poopPosition()), lights.flames()).withName("Routines.prepPoop()");
    }

    /**
     * Poops the note out of the intake.
     */
    public static Command poop() {
        return deadline(sequence(waitUntil(() -> !intake.hasNote()), waitSeconds(0.15)), intake.poop(), lights.flames())
            .withName("Routines.poop()");
    }

    /**
     * Returns the note from the feeder back to the intake.
     */
    public static Command handoff() {
        return sequence(
            parallel(
                pivot.goTo(PivotConstants.HANDOFF_POSITION),
                sequence(deadline(waitUntil(pivot::isSafeForIntake), intake.maintainPosition()), intake.handoffPosition())
            ),
            deadline(sequence(waitUntil(() -> intake.hasNote() && !feeder.hasNote()), waitSeconds(0.1)), feeder.barf(), intake.handoff())
        );
    }

    /**
     * Feeds notes directly from the intake straight into the shooter.
     * @param rampSpeed The speed to ramp the shooter by in percent duty cycle / second.
     */
    public static Command feedThrough(Supplier<Double> rampSpeed) {
        return parallel(safeArm(intake.intake()), feeder.shoot(true), shooter.driveManual(rampSpeed), lights.flames())
            .withName("Routines.feedThrough()");
    }

    /**
     * Juggles a note.
     */
    public static Command juggle() {
        return parallel(
            sequence(
                safeArm(intake.downPosition()),
                deadline(pivot.goTo(PivotConstants.MAX_POS), intake.maintainPosition()),
                parallel(
                    repeatingSequence(
                        deadline(sequence(waitSeconds(0.2), feeder.shoot()), shooter.setSpeed(0.2)),
                        deadline(waitUntil(feeder::hasNote).andThen(waitSeconds(0.1)), shooter.intakeHuman(), feeder.intakeHuman())
                    ),
                    intake.maintainPosition()
                )
            ),
            lights.flames()
        )
            .withName("Routines.juggle()");
    }

    /**
     * Juggles a note (differently).
     */
    public static Command juggle2() {
        return parallel(
            repeatingSequence(
                handoff(),
                intake.juggleHandoffPosition(),
                pivot.goTo(PivotConstants.MAX_POS),
                deadline(
                    waitUntil(feeder::hasNote).andThen(waitSeconds(0.1)),
                    intake.juggleHandoff(),
                    shooter.intakeHuman(),
                    feeder.intakeHuman()
                )
            ),
            lights.flames()
        )
            .withName("Routines.juggle()");
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
