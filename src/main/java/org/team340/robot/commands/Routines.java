package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
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
     * Intakes from the ground. Ends when the note is seated.
     */
    public Command intake() {
        return sequence(
            parallel(intake.apply(IntakeState.kIntake), feeder.apply(FeederSpeed.kReceive)),
            deadline(feeder.seat(), intake.apply(IntakeState.kIntake))
        ).withName("Routines.intake()");
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
            parallel(
                sequence(feeder.apply(FeederSpeed.kBarfForward).until(feeder::noNote), feeder.seat()),
                pivot.apply(PivotPosition.kDown)
            )
        ).withName("Routines.humanLoad()");
    }

    /**
     * Prepares to score in speaker. Does not end.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command prepSpeaker(Supplier<Double> x, Supplier<Double> y) {
        return parallel(
            swerve.driveSpeaker(x, y),
            pivot.targetSpeaker(swerve::getSpeakerDistance),
            shooter.targetSpeaker(swerve::getSpeakerDistance)
        ).withName("Routines.prepSpeaker()");
    }

    /**
     * Prepares to score in amp. Does not end.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command prepAmp(Supplier<Double> x, Supplier<Double> y) {
        return parallel(
            swerve.driveAmp(x, y),
            pivot.apply(PivotPosition.kAmp),
            shooter.apply(ShooterSpeed.kAmp)
        ).withName("Routines.prepSpeaker()");
    }

    /**
     * Prepares to feed a note to our alliance. Does not end.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command prepFeed(Supplier<Double> x, Supplier<Double> y) {
        return parallel(swerve.driveFeed(x, y), pivot.apply(PivotPosition.kFeed), shooter.apply(ShooterSpeed.kFeed));
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
            deadline(feeder.seat(), pivot.apply(PivotPosition.kDown))
        ).withName("Routines.fixDeadzone()");
    }

    /**
     * Barfs forwards (towards the intake). Does not end.
     */
    public Command barfForward() {
        return parallel(
            feeder.apply(FeederSpeed.kBarfForward),
            intake.apply(IntakeState.kBarf),
            shooter.apply(ShooterSpeed.kBarfForward)
        ).withName("Routines.barfForward()");
    }

    /**
     * Barfs backwards (towards the shooter). Does not end.
     */
    public Command barfBackward() {
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
