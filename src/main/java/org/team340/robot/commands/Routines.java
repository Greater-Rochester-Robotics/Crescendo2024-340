package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.function.Supplier;
import org.team340.robot.RobotContainer;
import org.team340.robot.subsystems.Amplifier;
import org.team340.robot.subsystems.Amplifier.AmplifierPosition;
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

    private final Amplifier amplifier;
    private final Feeder feeder;
    private final Intake intake;
    private final Pivot pivot;
    private final Shooter shooter;
    private final Swerve swerve;

    public Routines(RobotContainer robotContainer) {
        amplifier = robotContainer.amplifier;
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
            amplifier.apply(AmplifierPosition.kExtend),
            pivot.apply(PivotPosition.kAmp),
            shooter.apply(ShooterSpeed.kAmp),
            swerve.driveAmp(x, y)
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
