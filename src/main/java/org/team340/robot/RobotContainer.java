package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team340.lib.GRRDashboard;
import org.team340.lib.controller.Controller2;
import org.team340.lib.util.config.rev.RevConfigRegistry;
import org.team340.robot.Constants.ControllerConstants;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Climber;
import org.team340.robot.subsystems.Feeder;
import org.team340.robot.subsystems.Intake;
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
    private static Controller2 coDriver;

    public static Climber climber;
    public static Feeder feeder;
    public static Intake intake;
    public static Pivot pivot;
    public static Shooter shooter;
    public static Swerve swerve;

    /**
     * Entry to initializing subsystems and command execution.
     */
    public static void init() {
        // Initialize controllers.
        driver = new Controller2(ControllerConstants.DRIVER);
        coDriver = new Controller2(ControllerConstants.CO_DRIVER);

        // Add controllers to the dashboard.
        driver.addToDashboard();
        coDriver.addToDashboard();

        // Initialize subsystems.
        // climber = new Climber();
        feeder = new Feeder();
        intake = new Intake();
        pivot = new Pivot();
        shooter = new Shooter();
        // swerve = new Swerve();

        // Add subsystems to the dashboard.
        // climber.addToDashboard();
        feeder.addToDashboard();
        intake.addToDashboard();
        pivot.addToDashboard();
        shooter.addToDashboard();
        // swerve.addToDashboard();

        // Set systems check command.
        // GRRDashboard.setSystemsCheck(SystemsCheck.command());

        // Complete REV hardware initialization.
        RevConfigRegistry.burnFlash();
        RevConfigRegistry.printError();

        // Configure bindings and autos.
        configBindings();
        configAutos();
    }

    /**
     * This method should be used to declare triggers (created with an
     * arbitrary predicate or from controllers) and their bindings.
     */
    private static void configBindings() {
        // Set default commands.
        pivot.setDefaultCommand(pivot.maintainPosition());
        intake.setDefaultCommand(intake.maintainPosition());
        // swerve.setDefaultCommand(swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, true));

        Routines.onDisable().schedule();
        RobotModeTriggers.disabled().whileTrue(waitSeconds(6.0).andThen(Routines.onDisable()));

        /**
         * Driver bindings.
         */

        // POV Left => Zero swerve
        // driver.povLeft().onTrue(swerve.zeroIMU(Math2.ROTATION2D_0));

        driver.rightBumper().whileTrue(pivot.goToAngle(() -> Math.toRadians(16.5)));
        driver.leftBumper().whileTrue(pivot.home(true));

        driver.a().whileTrue(Routines.intake()).onFalse(feeder.seatNote());
        driver.b().whileTrue(intake.retract());

        driver.x().whileTrue(feeder.shootNote());
        driver.y().toggleOnTrue(shooter.setSpeed(8000.0));

        driver.povUp().onTrue(feeder.seatNote());

        // driver.a().whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        // driver.x().whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));
        // driver.b().whileTrue(swerve.sysIdDynamic(Direction.kForward));
        // driver.y().whileTrue(swerve.sysIdDynamic(Direction.kReverse));

        /**
         * Co-driver bindings.
         */

        // A => Do nothing
        coDriver.a().onTrue(none());
    }

    /**
     * Autonomous commands should be declared here and
     * added to {@link GRRDashboard}.
     */
    private static void configAutos() {}

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
        return driver.getTriggerDifference(ControllerConstants.DRIVE_ROT_MULTIPLIER, ControllerConstants.DRIVE_ROT_EXP);
    }
}
