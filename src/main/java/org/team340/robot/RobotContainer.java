package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team340.lib.controller.Controller;
import org.team340.lib.util.rev.RevConfigRegistry;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Feeder;
import org.team340.robot.subsystems.Feeder.FeederSpeed;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Intake.IntakeState;
import org.team340.robot.subsystems.Pivot;
import org.team340.robot.subsystems.Pivot.PivotPosition;
import org.team340.robot.subsystems.Shooter;
import org.team340.robot.subsystems.Swerve;

/**
 * This class is used to declare subsystems, commands, and trigger mappings.
 */
@Logged
public final class RobotContainer {

    private final Controller driver;
    private final Controller coDriver;

    public final Feeder feeder;
    public final Intake intake;
    // public Lights lights;
    public final Pivot pivot;
    public final Shooter shooter;
    public final Swerve swerve;

    public final Routines routines;

    /**
     * Entry to initializing subsystems and command execution.
     */
    public RobotContainer() {
        // Initialize controllers.
        driver = new Controller(Constants.kDriver);
        coDriver = new Controller(Constants.kCoDriver);

        // Initialize subsystems.
        feeder = new Feeder();
        intake = new Intake();
        // lights = new Lights();
        pivot = new Pivot();
        shooter = new Shooter();
        swerve = new Swerve();

        routines = new Routines(this);

        // Complete REV hardware initialization.
        RevConfigRegistry.burnFlashAll();

        // Configure bindings and autos.
        configBindings();
    }

    /**
     * This method should be used to declare triggers (created with an
     * arbitrary predicate or from controllers) and their bindings.
     */
    private void configBindings() {
        // Set default commands.
        intake.setDefaultCommand(intake.apply(IntakeState.kRetract));
        // lights.setDefaultCommand(lights.run(feeder::hasNote));
        pivot.setDefaultCommand(pivot.apply(PivotPosition.kDown));
        swerve.setDefaultCommand(swerve.drive(driver::getLeftX, driver::getLeftY, driver::getTriggerDifference));

        routines.onDisable().schedule();
        RobotModeTriggers.disabled().onTrue(routines.onDisable());

        /**
         * Driver bindings.
         */

        // A => Intake (Hold)
        driver.a().whileTrue(routines.intake());

        // B => Intake from Human Player (Hold)
        driver.b().whileTrue(routines.humanLoad());

        // Y => Shoot (Hold)
        driver.y().whileTrue(feeder.apply(FeederSpeed.kShoot));

        driver.leftBumper().whileTrue(pivot.manual(() -> -driver.getRightY()));

        driver
            .rightBumper()
            .whileTrue(parallel(shooter.manual(() -> -driver.getRightY() * 1000.0), pivot.manual(() -> 0.0)));

        // Right Stick Up => Barf (Hold)
        driver
            .rightJoystickUp()
            .or(driver.rightJoystickDown())
            .and(driver.leftBumper().or(driver.rightBumper()).negate())
            .whileTrue(routines.barf());

        // POV Down => Pivot home (Tap)
        driver.povDown().onTrue(pivot.home(true));

        // POV Left => Zero swerve (Tap)
        driver.povLeft().onTrue(swerve.tareRotation());

        /**
         * Co-driver bindings.
         */

        coDriver
            .leftBumper()
            .and(coDriver.rightBumper())
            .whileTrue(
                parallel(
                    intake.manual(() -> -coDriver.getLeftY() * 0.5),
                    pivot.manual(() -> -coDriver.getRightY() * 0.4),
                    shooter.manual(() -> -coDriver.getRightX() * 500.0)
                ).withName("coDriver.leftBumper().and(coDriver.rightBumper()).whileTrue()")
            );

        // A => Reserved for climb
        coDriver.a().whileTrue(none());

        // POV Up => Fix Deadzone (Hold)
        coDriver.povUp().whileTrue(routines.fixDeadzone());

        // POV Down => Pivot Home (Hold)
        coDriver.povDown().whileTrue(pivot.home(true));

        // Back / Start => he he rumble rumble
        coDriver.back().toggleOnTrue(setCoDriverRumble(RumbleType.kLeftRumble, 0.5).ignoringDisable(true));
        coDriver.start().toggleOnTrue(setCoDriverRumble(RumbleType.kRightRumble, 0.5).ignoringDisable(true));
    }

    /**
     * Sets the rumble on the Co-Driver's controller.
     * @param type The rumble type.
     * @param value The normalized value to set the rumble to ({@code 0.0} to {@code 1.0}).
     */
    private Command setCoDriverRumble(RumbleType type, double value) {
        return runEnd(() -> coDriver.getHID().setRumble(type, value), () -> coDriver.getHID().setRumble(type, 0.0));
    }
}
