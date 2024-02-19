package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team340.lib.GRRDashboard;
import org.team340.lib.controller.Controller2;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.rev.RevConfigRegistry;
import org.team340.robot.Constants.ControllerConstants;
import org.team340.robot.commands.Autos;
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
        climber = new Climber();
        feeder = new Feeder();
        intake = new Intake();
        pivot = new Pivot();
        shooter = new Shooter();
        swerve = new Swerve();

        // Add subsystems to the dashboard.
        climber.addToDashboard();
        feeder.addToDashboard();
        intake.addToDashboard();
        pivot.addToDashboard();
        shooter.addToDashboard();
        swerve.addToDashboard();

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
        // shooter.setDefaultCommand(shooter.setSpeed(3000));
        pivot.setDefaultCommand(pivot.maintainPosition());
        intake.setDefaultCommand(intake.maintainPosition());
        swerve.setDefaultCommand(swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, true));

        Routines.onDisable().schedule();
        RobotModeTriggers.disabled().whileTrue(Routines.onDisable());

        /**
         * Driver bindings.
         */

        // A => Intake (Tap = Down, Hold = Run roller)
        driver.a().whileTrue(Routines.intake()).onFalse(parallel(feeder.seatNote(), intake.intakeDown()));

        // B => Intake Safe Position (Tap)
        driver
            .b()
            .onTrue(Routines.intakeFromHuman())
            .onFalse(
                parallel(
                    shooter.setSpeed(0).withTimeout(2.0),
                    parallel(pivot.goToAngle(0.0), waitUntil(pivot::isSafeForIntake)).andThen(intake.intakeDown())
                )
            );

        // X => Amp Score (Hold)
        driver
            .x()
            .onTrue(Routines.prepScoreAmp(RobotContainer::getDriveX, RobotContainer::getDriveY))
            .onFalse(intake.scoreAmp().withTimeout(1.5));

        // Y => Shoot (Tap)
        driver.y().whileTrue(feeder.shootNote());

        driver.rightJoystickUp().onTrue(Routines.protectIntake());
        driver.rightJoystickDown().onTrue(intake.intakeDown());

        // Right Bumper => Target Speaker (Hold)
        driver
            .rightBumper()
            .whileTrue(
                parallel(
                    swerve.driveOnTarget(RobotContainer::getDriveX, RobotContainer::getDriveY),
                    Routines.prepShootSpeaker(swerve::getDistanceToSpeaker)
                )
            );

        // Left Bumper => Face Stage (Toggle)
        driver.leftBumper().whileTrue(pivot.home(true));
        // driver.leftBumper().whileTrue(swerve.alignWithStage(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // POV Up => Barf Backwards
        driver.povUp().whileTrue(Routines.spitBack());

        // POV Down => Barf Forward
        driver.povDown().whileTrue(Routines.spitFront());

        // POV Left => Zero swerve
        driver.povLeft().onTrue(swerve.zeroIMU(Math2.ROTATION2D_0));
        driver.povRight().whileTrue(pivot.goToAngle(() -> Math.toRadians(15.0)));
        driver.start().whileTrue(climber.zeroArms());
        driver.back().whileTrue(climber.toPosition(0.0)); //feeder.shootNote());
        // driver.rightBumper().whileTrue(pivot.goToAngle(() -> Math.toRadians(55.0)));
        // driver.a().whileTrue(Routines.intake()).whileFalse(parallel(feeder.seatNote(), intake.intakeDown()));
        // driver.y().whileTrue(feeder.shootNote());
        // driver.x().toggleOnTrue(shooter.setSpeed(6000.0));

        Trigger driverNotLock = driver
            .a()
            .negate()
            .and(
                driver
                    .b()
                    .negate()
                    .and(driver.x().negate().and(driver.y().negate().and(driver.povUp().negate().and(driver.povDown().negate()))))
            );

        /**
         * Co-driver bindings.
         */

        // A => Overrides
        coDriver.a().onTrue(none());

        // B => Reserved
        coDriver.b().onTrue(none());

        // X => Reserved For Climb
        coDriver.x().onTrue(none());

        // Y => Reserved For Climb
        coDriver.x().onTrue(none());
        /**
         * SysId Routines
         */

        // driver.a().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driver.b().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driver.x().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driver.y().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // driver.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driver.b().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driver.x().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driver.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Autonomous commands should be declared here and
     * added to {@link GRRDashboard}.
     */
    private static void configAutos() {
        var fourPiece = Choreo.getTrajectoryGroup("FourPiece");
        GRRDashboard.addAutoCommand("Four Piece", fourPiece, Autos.fourPiece(fourPiece));
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
        return driver.getTriggerDifference(ControllerConstants.DRIVE_ROT_MULTIPLIER, ControllerConstants.DRIVE_ROT_EXP);
    }
}
