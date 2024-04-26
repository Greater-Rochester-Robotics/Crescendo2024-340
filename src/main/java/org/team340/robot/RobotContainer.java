package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team340.lib.controller.Controller2;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.rev.RevConfigRegistry;
import org.team340.robot.Constants.ControllerConstants;
import org.team340.robot.Constants.PivotConstants;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Feeder;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Lights;
import org.team340.robot.subsystems.Pivot;
import org.team340.robot.subsystems.Shooter;
import org.team340.robot.subsystems.Swerve;

/**
 * This class is used to declare subsystems, commands, and trigger mappings.
 */
public final class RobotContainer {

    private RobotContainer() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static Controller2 driver;

    public static Feeder feeder;
    public static Intake intake;
    public static Lights lights;
    public static Pivot pivot;
    public static Shooter shooter;
    public static Swerve swerve;

    /**
     * Entry to initializing subsystems and command execution.
     */
    public static void init() {
        // Initialize controllers.
        driver = new Controller2(ControllerConstants.DRIVER);

        // Add controllers to the dashboard.
        driver.addToDashboard();

        // Initialize subsystems.
        feeder = new Feeder();
        intake = new Intake();
        lights = new Lights();
        pivot = new Pivot();
        shooter = new Shooter();
        swerve = new Swerve();

        // Add subsystems to the dashboard.
        feeder.addToDashboard();
        intake.addToDashboard();
        lights.addToDashboard();
        pivot.addToDashboard();
        shooter.addToDashboard();
        swerve.addToDashboard();

        // Complete REV hardware initialization.
        RevConfigRegistry.burnFlash();
        RevConfigRegistry.printError();

        // Configure bindings.
        configBindings();
    }

    /**
     * This method should be used to declare triggers (created with an
     * arbitrary predicate or from controllers) and their bindings.
     */
    private static void configBindings() {
        // Set default commands.
        intake.setDefaultCommand(intake.maintainPosition());
        lights.setDefaultCommand(lights.idle());
        pivot.setDefaultCommand(pivot.maintainPosition());
        swerve.setDefaultCommand(swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, true));

        Routines.onDisable().schedule();
        RobotModeTriggers.disabled().onTrue(Routines.onDisable());

        /**
         * Driver bindings.
         */

        Trigger bumpersReleased = driver.leftBumper().or(driver.rightBumper()).negate();

        // A => Intake (Tap = Down, Hold = Run roller)
        driver.a().whileTrue(Routines.intake()).onFalse(Routines.finishIntake());

        // B => Intake from Human Player (Hold)
        driver.b().onTrue(Routines.intakeHuman()).onFalse(Routines.finishIntakeHuman());

        // X => Poop (Hold)
        driver.x().whileTrue(Routines.prepPoop()).onFalse(Routines.poop());

        // Y => Shoot (Tap)
        driver.y().whileTrue(feeder.shoot());

        // Right Joystick Up + Pressed => Intake upright
        driver.rightJoystickUp().and(driver.rightStick()).and(bumpersReleased).onTrue(Routines.safeArm(intake.uprightPosition()));

        // Right Joystick Down + Pressed => Intake safe position
        driver.rightJoystickDown().and(driver.rightStick()).and(bumpersReleased).onTrue(Routines.safeArm(intake.safePosition()));

        // POV Up => Barf (Hold)
        driver.povUp().whileTrue(Routines.barf()).onFalse(Routines.safeArm(intake.safePosition()));

        // POV Down => Pivot Down (Tap)
        driver.povDown().onTrue(pivot.goTo(PivotConstants.DOWN_POSITION));

        // POV Left => Zero swerve
        driver.povLeft().onTrue(swerve.zeroIMU(Math2.ROTATION2D_0));

        // POV Right => Toggle feed through
        driver.povRight().toggleOnTrue(Routines.feedThrough(RobotContainer::getShooterManual));

        // Left Bumper => Change pivot position manually (Hold)
        driver.leftBumper().and(driver.rightBumper().negate()).whileTrue(pivot.driveManual(RobotContainer::getPivotManual));

        // Right Bumper => Shoot with manual speed adjustment (Hold)
        driver
            .rightBumper()
            .and(driver.leftBumper().negate())
            .whileTrue(parallel(shooter.driveManual(RobotContainer::getShooterManual), lights.flames()));

        // Back => Juggle (Hold)
        driver.back().whileTrue(Routines.juggle()).onFalse(Routines.finishIntakeHuman());

        // Start => Juggle 2 (Hold)
        driver.start().whileTrue(Routines.juggle2()).onFalse(Routines.finishIntakeHuman());
    }

    /**
     * Gets the X axis drive speed from the driver's controller.
     */
    private static double getDriveX() {
        double multiplier =
            ((driver.getHID().getLeftStickButton()) ? ControllerConstants.DRIVE_MULTIPLIER_MODIFIED : ControllerConstants.DRIVE_MULTIPLIER);
        return -driver.getLeftY(multiplier, ControllerConstants.DRIVE_EXP);
    }

    /**
     * Gets the Y axis drive speed from the driver's controller.
     */
    private static double getDriveY() {
        double multiplier =
            ((driver.getHID().getLeftStickButton()) ? ControllerConstants.DRIVE_MULTIPLIER_MODIFIED : ControllerConstants.DRIVE_MULTIPLIER);
        return -driver.getLeftX(multiplier, ControllerConstants.DRIVE_EXP);
    }

    /**
     * Gets the rotational drive speed from the driver's controller.
     */
    private static double getDriveRotate() {
        double multiplier =
            (
                (driver.getHID().getLeftStickButton())
                    ? ControllerConstants.DRIVE_ROT_MULTIPLIER_MODIFIED
                    : ControllerConstants.DRIVE_ROT_MULTIPLIER
            );
        return driver.getTriggerDifference(multiplier, ControllerConstants.DRIVE_ROT_EXP);
    }

    /**
     * Gets the manual pivot adjustment speed from the driver's controller in radians/second.
     */
    private static double getPivotManual() {
        return driver.getRightY() * -1.1;
    }

    /**
     * Gets the manual shooter adjustment speed from the driver's controller in percent duty cycle / second.
     */
    private static double getShooterManual() {
        return driver.getRightY() * -0.5;
    }
}
