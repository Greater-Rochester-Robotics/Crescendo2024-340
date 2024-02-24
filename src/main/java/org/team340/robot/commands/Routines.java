package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.Mutable;
import org.team340.robot.Constants;

// TODO Discuss bindings and what commands are needed

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

    public static Command intakeOverride() {
        return parallel(intake.intakeOverride(), feeder.shoot()).withName("Routines.intakeOverride()");
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
     * Scores in the amp.
     */
    public static Command scoreAmp() {
        Mutable<Boolean> approached = new Mutable<>(false);
        return sequence(
            runOnce(() -> approached.set(false)),
            parallel(
                swerve.approachAmp().finallyDo(() -> approached.set(true)),
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
                    intake.maintainPosition().until(approached::get)
                )
            ),
            parallel(swerve.scoreAmp(), sequence(waitSeconds(1.3), intake.scoreAmp()))
        )
            .withName("Routines.scoreAmp()");
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
