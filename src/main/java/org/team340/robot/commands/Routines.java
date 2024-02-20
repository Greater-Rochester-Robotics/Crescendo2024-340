package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
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
        return sequence(
            waitUntil(pivot::isSafeForIntake),
            intake.intakeDown(),
            race(
                // sequence(waitUntil(intake::hasNote), waitUntil(() -> !intake.hasNote())),
                feeder.receiveNote(),
                intake.intake()
            ),
            feeder.seatNote()
        )
            .withName("Routines.intake()");
    }

    /**
     * Moves the intake to its safe position inside the bumpers. Moves the pivot if needed.
     */
    public static Command retractIntake() {
        return sequence(
            either(
                none().withName("pivot.isSafeFallthrough"),
                pivot.goToAngle(Constants.PivotConstants.SAFE_FOR_INTAKE_ANGLE),
                pivot::isSafeForIntake
            ),
            intake.retract()
        )
            .withName("Routines.retractIntake()");
    }

    public static Command intakeFromHuman() {
        return parallel(
            sequence(
                deadline(
                    sequence(waitUntil(feeder::hasNote), waitUntil(() -> !feeder.hasNote())),
                    parallel(shooter.intakeFromHuman(), feeder.intakeFromHuman())
                ),
                feeder.seatNote()
            ),
            sequence(
                pivot.goToAngle(Constants.PivotConstants.SAFE_FOR_INTAKE_ANGLE).unless(pivot::isSafeForIntake),
                intake.toUprightPosition(),
                pivot.goToAngle(Constants.PivotConstants.MAXIMUM_ANGLE)
            )
        )
            .withName("Routines.intakeFromHuman()");
    }

    /**
     * This raises the intake in preparation for scoring in the amp.
     */
    public static Command prepScoreAmp() {
        return parallel(
            sequence(
                sequence(
                    parallel(
                        pivot.goToAngle(Constants.PivotConstants.OPTIMAL_RECEIVE_NOTE_ANGLE),
                        sequence(waitUntil(pivot::isSafeForIntake), intake.intakeDown())
                    ),
                    sequence(
                        intake.intakeDown(),
                        deadline(
                            sequence(waitUntil(() -> intake.hasNote() && !feeder.hasNote()), waitSeconds(0.1)),
                            shooterFeederSpitFront(),
                            intake.receiveFromShooter()
                        )
                    )
                )
                    .unless(intake::hasNote),
                intake.scoreAmpPosition(),
                intake.maintainPosition()
            ),
            swerve.driveAmp()
        )
            .withName("prepScoreAmp()");
    }

    /**
     * This starts the shooter and adjusts pivot angle in preparation for shooting.
     * This command does not end of it's own accord.
     * @param distance This is a supplier of the robots position.
     */
    public static Command prepShootSpeaker(Supplier<Double> distance) {
        // return parallel(shooter.setSpeedWithDist(distance)).withName("Routines.prepShootSpeaker()");
        return parallel(shooter.setSpeedWithDist(distance), pivot.goToAngleWithDist(distance)).withName("Routines.prepShootSpeaker()");
    }

    /**
     * This command takes the current position, uses it to decide the angle to shoot at, then shoots the note.
     * Note that this will not correct the robot's yaw or position.
     * @param distance This is the position used for the math, note that it must be a supplier.
     * @return This command.
     */
    public static Command shootSpeaker(Supplier<Double> distance) {
        return parallel(
            prepShootSpeaker(distance),
            //TODO: add drive on target condition to move forward
            sequence(waitUntil(() -> shooter.hasReachedSpeed() && pivot.isOnTarget()), feeder.shootNote())
        )
            .withName("Routines.shootSpeaker()");
    }

    /**
     * This command spits the note regardless of if it's in the shooter or intake.
     * @return This command.
     */
    public static Command spitFront() {
        return sequence(
            parallel(pivot.goToAngle(Constants.PivotConstants.SPIT_ANGLE), intake.toSpitPosition()),
            parallel(shooterFeederSpitFront(), intake.spit())
        )
            .withName("Routines.spitFront()");
    }

    /**
     * Designed to fully barf and designed for handoff back to intake.
     * @return This command.
     */
    private static Command shooterFeederSpitFront() {
        return parallel(shooter.spitFront(), feeder.spitFront()).withName("Routines.shooterFeederSpitFront()");
    }

    public static Command spitBack() {
        return parallel(shooter.spitBack(), sequence(waitSeconds(0.25), parallel(feeder.spitBack(), intake.intake())))
            .withName("Routines.spitBack()");
    }

    /**
     * This command sets the pivot motors to coast mode, and then back to break mode after it ends,
     * it should be called when the robot is disabled.
     */
    public static Command onDisable() {
        return sequence(waitSeconds(6.0), parallel(feeder.onDisable(), intake.onDisable(), pivot.onDisable(), climber.onDisable()))
            .withName("Routines.onDisable()");
    }
}
