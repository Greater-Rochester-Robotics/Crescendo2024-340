package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team340.lib.GRRDashboard;
import org.team340.lib.controller.Controller2;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.rev.RevConfigRegistry;
import org.team340.robot.Constants.ControllerConstants;
import org.team340.robot.Constants.FieldPositions;
import org.team340.robot.Constants.PivotConstants;
import org.team340.robot.commands.Autos;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Climber;
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
    private static Controller2 coDriver;

    public static Climber climber;
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
        coDriver = new Controller2(ControllerConstants.CO_DRIVER);

        // Add controllers to the dashboard.
        driver.addToDashboard();
        coDriver.addToDashboard();

        // Initialize subsystems.
        climber = new Climber();
        feeder = new Feeder();
        intake = new Intake();
        lights = new Lights();
        pivot = new Pivot();
        shooter = new Shooter();
        swerve = new Swerve();

        // Add subsystems to the dashboard.
        climber.addToDashboard();
        feeder.addToDashboard();
        intake.addToDashboard();
        lights.addToDashboard();
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
        lights.setDefaultCommand(lights.defaultCommand(intake::hasNote, feeder::hasNote));
        pivot.setDefaultCommand(pivot.maintainPosition());
        shooter.setDefaultCommand(shooter.targetDistance(swerve::getSpeakerDistance, 3500.0, swerve::inOpponentWing));
        swerve.setDefaultCommand(swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, true));

        Routines.onDisable().schedule();
        RobotModeTriggers.disabled().onTrue(Routines.onDisable());

        /**
         * Driver bindings.
         */

        // A => Intake (Tap = Down, Hold = Run roller)
        driver.a().whileTrue(Routines.intake()).onFalse(Routines.finishIntake());

        // B => Intake from Human Player (Hold)
        driver.b().onTrue(Routines.intakeHuman(RobotContainer::getDriveX, RobotContainer::getDriveY)).onFalse(Routines.finishIntakeHuman());

        // X => Prep Amp (Hold)
        driver.x().whileTrue(Routines.scoreAmp(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // Y => Shoot (Tap)
        driver.y().whileTrue(feeder.shoot());

        // Right Joystick Up => Protect intake
        driver.rightJoystickUp().onTrue(intake.retractPosition());

        // Right Joystick Down => Intake down
        driver.rightJoystickDown().onTrue(intake.safePosition());

        // Right Bumper => Prep Speaker (Hold)
        driver.rightBumper().whileTrue(Routines.prepSpeaker(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // Left Bumper => Prep Feed (Hold)
        driver.leftBumper().whileTrue(Routines.prepFeed(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // POV Up => Barf Forwards (Hold)
        driver.povUp().whileTrue(Routines.barfForward());

        // POV Down => Pivot Down (Tap)
        driver.povDown().onTrue(pivot.goTo(PivotConstants.DOWN_POSITION));

        // POV Left => Zero swerve
        driver.povLeft().onTrue(swerve.zeroIMU(Math2.ROTATION2D_0));

        // POV Right => Prep Climb (Hold)
        driver.povRight().whileTrue(Routines.prepClimb(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // Start => Toggle Shooter
        driver.start().toggleOnTrue(shooter.setSpeed(0.0));

        // Back => Dump Odometry
        driver.back().onTrue(swerve.dumpOdometry());

        /**
         * Co-driver bindings.
         */

        coDriver
            .leftBumper()
            .and(coDriver.rightBumper())
            .whileTrue(
                parallel(
                    intake.driveArmManual(() -> -coDriver.getLeftY() * 0.5),
                    pivot.driveManual(() -> -coDriver.getRightY() * 0.4),
                    shooter.driveManual(() -> -coDriver.getRightX() * 500.0)
                )
                    .withName("coDriver.leftBumper().and(coDriver.rightBumper()).whileTrue()")
            );

        // A => Climb (Hold)
        coDriver.a().whileTrue(climber.climb(swerve::getRoll));

        // B => Overrides intake (Hold)
        coDriver.b().whileTrue(Routines.intakeOverride());

        // X => Fender Shot (Hold)
        coDriver.x().whileTrue(pivot.targetDistance(() -> FieldPositions.FENDER_SHOT_DISTANCE));

        // Y => Fix deadzone (Hold)
        coDriver.y().onTrue(Routines.fixDeadzone());

        // Both Triggers => Drives climber manually
        coDriver
            .leftTrigger()
            .and(coDriver.rightTrigger())
            .whileTrue(climber.driveManual(() -> -coDriver.getLeftY() * 0.3, () -> -coDriver.getRightY() * 0.3));

        // POV Up => Prep Speaker (Hold)
        coDriver.povUp().whileTrue(Routines.prepSpeaker(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // POV Down => Pivot home (Hold)
        coDriver.povDown().whileTrue(pivot.home(true));

        // Back / Start => he he rumble rumble
        coDriver.back().toggleOnTrue(setCoDriverRumble(RumbleType.kLeftRumble, 0.5).ignoringDisable(true));
        coDriver.start().toggleOnTrue(setCoDriverRumble(RumbleType.kRightRumble, 0.5).ignoringDisable(true));
    }

    /**
     * Autonomous commands should be declared here and
     * added to {@link GRRDashboard}.
     */
    private static void configAutos() {
        var fourPieceClose = Choreo.getTrajectoryGroup("FourPieceClose");
        GRRDashboard.addAutoCommand("Four Piece Close", fourPieceClose, Autos.fourPieceClose(fourPieceClose));

        var fivePieceAmp = Choreo.getTrajectoryGroup("FivePieceAmp");
        GRRDashboard.addAutoCommand("Five Piece Amp", fivePieceAmp, Autos.fivePieceAmp(fivePieceAmp));

        var fourPieceFar = Choreo.getTrajectoryGroup("FourPieceFar");
        GRRDashboard.addAutoCommand("Four Piece Far", fourPieceFar, Autos.fourPieceFar(fourPieceFar));

        var fourPieceSource12 = Choreo.getTrajectoryGroup("FourPieceSource12");
        GRRDashboard.addAutoCommand("Four Piece Source: 1, 2", fourPieceSource12, Autos.fourPieceSource(fourPieceSource12));

        var fourPieceSource13 = Choreo.getTrajectoryGroup("FourPieceSource13");
        GRRDashboard.addAutoCommand("Four Piece Source: 1, 3", fourPieceSource13, Autos.fourPieceSource(fourPieceSource13));

        var fourPieceSource21 = Choreo.getTrajectoryGroup("FourPieceSource21");
        GRRDashboard.addAutoCommand("Four Piece Source: 2, 1", fourPieceSource21, Autos.fourPieceSource(fourPieceSource21));

        var fourPieceSource23 = Choreo.getTrajectoryGroup("FourPieceSource23");
        GRRDashboard.addAutoCommand("Four Piece Source: 2, 3", fourPieceSource23, Autos.fourPieceSource(fourPieceSource23));

        var fourPieceSource31 = Choreo.getTrajectoryGroup("FourPieceSource31");
        GRRDashboard.addAutoCommand("Four Piece Source: 3, 1", fourPieceSource31, Autos.fourPieceSource(fourPieceSource31));

        var fourPieceSource32 = Choreo.getTrajectoryGroup("FourPieceSource32");
        GRRDashboard.addAutoCommand("Four Piece Source: 3, 2", fourPieceSource32, Autos.fourPieceSource(fourPieceSource32));
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
