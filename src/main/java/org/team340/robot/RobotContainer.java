package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team340.lib.GRRDashboard;
import org.team340.lib.controller.Controller2;
import org.team340.lib.util.Math2;
import org.team340.lib.util.TriggerLockout;
import org.team340.lib.util.config.rev.RevConfigRegistry;
import org.team340.robot.Constants.ControllerConstants;
import org.team340.robot.Constants.FieldPositions;
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
        intake.setDefaultCommand(intake.maintainPosition());
        pivot.setDefaultCommand(pivot.maintainPosition());
        shooter.setDefaultCommand(shooter.targetDistance(swerve::getSpeakerDistance, 3500.0, swerve::inOpponentWing));
        swerve.setDefaultCommand(swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, true));

        Routines.onDisable().schedule();
        RobotModeTriggers.disabled().whileTrue(Routines.onDisable());

        /**
         * Driver bindings.
         */

        // A => Intake (Tap = Down, Hold = Run roller)
        driver
            .a()
            .whileTrue(Routines.intake())
            .onFalse(parallel(feeder.seat(), intake.safePosition()).withName("RobotContainer.driver.a().onFalse()"));

        // B => Intake from Human Player (Hold)
        driver
            .b()
            .onTrue(Routines.intakeHuman())
            .onFalse(
                parallel(
                    shooter.setSpeed(0).withTimeout(2.0),
                    parallel(pivot.goTo(0.0), waitUntil(pivot::isSafeForIntake)).andThen(intake.safePosition())
                )
                    .withName("RobotContainer.driver.b().onFalse()")
            );

        // X => Amp Score (Hold)
        driver.x().whileTrue(Routines.prepAmp(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // Y => Shoot (Tap)
        driver.y().whileTrue(feeder.shoot());

        // Right Joystick Up => Protect intake
        driver.rightJoystickUp().onTrue(intake.retractPosition());

        // Right Joystick Down => Intake down
        driver.rightJoystickDown().onTrue(intake.safePosition());

        // Right Bumper => Target Speaker (Hold)
        driver
            .rightBumper()
            .whileTrue(
                parallel(
                    swerve.driveSpeaker(RobotContainer::getDriveX, RobotContainer::getDriveY),
                    pivot.targetDistance(swerve::getSpeakerDistance)
                )
                    .withName("RobotContainer.driver.rightBumper().whileTrue()")
            );

        // Left Bumper => Face Stage (Hold)
        driver.leftBumper().whileTrue(Routines.prepClimb(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // POV Up => Barf Backwards (Hold)
        driver.povUp().whileTrue(Routines.barfForward());

        // POV Down => Barf Forward (Hold)
        driver.povDown().whileTrue(Routines.barfBackward());

        // POV Left => Zero swerve
        driver.povLeft().onTrue(swerve.zeroIMU(Math2.ROTATION2D_0));

        // POV Right => Zero pivot (Hold)
        driver.povRight().onTrue(pivot.goTo(Math.toRadians(3.0)));

        // Start => Toggle Shooter
        driver.start().toggleOnTrue(shooter.setSpeed(0.0));

        // Back => Dump Odometry
        driver.back().onTrue(swerve.dumpOdometry());

        /**
         * Co-driver bindings.
         */

        Trigger coDriverOverride = TriggerLockout
            .of(
                driver.a(),
                driver.b(),
                // driver.x(),
                driver.y(),
                driver.rightJoystickUp(),
                driver.rightJoystickDown(),
                driver.povUp(),
                driver.povDown(),
                driver.povLeft(),
                driver.povRight(),
                driver.rightBumper()
            )
            .and(coDriver.leftBumper())
            .and(coDriver.rightBumper());

        coDriverOverride.whileTrue(
            parallel(
                intake.driveArmManual(() -> -coDriver.getLeftY() * 0.5),
                pivot.driveManual(() -> -coDriver.getRightY() * 0.4),
                shooter.driveManual(() -> -coDriver.getRightX() * 500.0)
            )
                .withName("RobotContainer.coDriverOverride.whileTrue()")
        );

        // A => Climb (Hold)
        coDriver.a().whileTrue(climber.climb(swerve::getRoll));

        // B => Overrides intake (Hold)
        coDriver.b().and(coDriverOverride).whileTrue(Routines.intakeOverride());

        // X => Fender Shot (Hold)
        coDriver.x().whileTrue(pivot.targetDistance(() -> FieldPositions.FENDER_SHOT_DISTANCE));

        // Y => Score Amp (Hold)
        coDriver.y().whileTrue(intake.scoreAmp());

        // Both Triggers => Drives climber manually
        coDriver
            .leftTrigger()
            .and(coDriver.rightTrigger())
            .whileTrue(climber.driveManual(() -> -coDriver.getLeftY() * 0.3, () -> -coDriver.getRightY() * 0.3));

        // POV => Pivot home (Hold)
        coDriver.povUp().whileTrue(pivot.home(true));

        // Back / Start => he he rumble rumble
        coDriver.back().toggleOnTrue(setCoDriverRumble(RumbleType.kLeftRumble, 0.5).ignoringDisable(true));
        coDriver.start().toggleOnTrue(setCoDriverRumble(RumbleType.kRightRumble, 0.5).ignoringDisable(true));
    }

    /**
     * Autonomous commands should be declared here and
     * added to {@link GRRDashboard}.
     */
    private static void configAutos() {
        var fourPieceLeft = Choreo.getTrajectoryGroup("FourPieceLeft");
        GRRDashboard.addAutoCommand("Four Piece Left", fourPieceLeft, Autos.fourPieceLeft(fourPieceLeft));

        var fourPieceRight = Choreo.getTrajectoryGroup("FourPieceRight");
        GRRDashboard.addAutoCommand("Four Piece Right", fourPieceRight, Autos.fourPieceRight(fourPieceRight));

        var fourPieceRight2 = Choreo.getTrajectoryGroup("FourPieceRight2");
        GRRDashboard.addAutoCommand("Four Piece Right 2.0", fourPieceRight2, Autos.fourPieceRight(fourPieceRight2));

        var fourPieceFront = Choreo.getTrajectoryGroup("FourPieceFront");
        GRRDashboard.addAutoCommand("Four Piece Front", fourPieceFront, Autos.fourPieceFront(fourPieceFront));

        var fourPieceFar = Choreo.getTrajectoryGroup("FourPieceFar");
        GRRDashboard.addAutoCommand("Four Piece Far", fourPieceFar, Autos.fourPieceFar(fourPieceFar));

        var fourPieceFar2 = Choreo.getTrajectoryGroup("FourPieceFar2");
        GRRDashboard.addAutoCommand("Four Piece Far 2.0", fourPieceFar2, Autos.fourPieceFar(fourPieceFar2));
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

    /**
     * Sets the rumble on the Co-Driver's controller.
     * @param type The rumble type.
     * @param value The normalized value to set the rumble to ({@code 0.0} to {@code 1.0}).
     */
    private static Command setCoDriverRumble(RumbleType type, double value) {
        return runEnd(() -> coDriver.getHID().setRumble(type, value), () -> coDriver.getHID().setRumble(type, 0.0));
    }
}
