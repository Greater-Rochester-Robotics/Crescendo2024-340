package org.team340.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveAPI.ForwardPerspective;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.util.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

/**
 * The swerve subsystem.
 */
@Logged
public class Swerve extends GRRSubsystem {

    private static final SwerveModuleConfig kFrontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(0.288925, 0.288925)
        .setMoveMotor(SwerveMotors.sparkFlex(RobotMap.kMoveFL, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkFlex(RobotMap.kTurnFL, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkFlexEncoder(0.3802, true));

    private static final SwerveModuleConfig kFrontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(0.288925, -0.288925)
        .setMoveMotor(SwerveMotors.sparkFlex(RobotMap.kMoveFR, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkFlex(RobotMap.kTurnFR, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkFlexEncoder(0.8859, true));

    private static final SwerveModuleConfig kBackLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-0.288925, 0.288925)
        .setMoveMotor(SwerveMotors.sparkFlex(RobotMap.kMoveBL, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkFlex(RobotMap.kTurnBL, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkFlexEncoder(0.3182, true));

    private static final SwerveModuleConfig kBackRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-0.288925, -0.288925)
        .setMoveMotor(SwerveMotors.sparkFlex(RobotMap.kMoveBR, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkFlex(RobotMap.kTurnBR, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkFlexEncoder(0.6675, true));

    public static final SwerveConfig kConfig = new SwerveConfig()
        .setTimings(Constants.kPeriod, Constants.kPeriod, 0.04)
        .setMovePID(0.01, 0.0, 0.0, 0.0)
        .setMoveFF(0.0221, 0.1068)
        .setTurnPID(0.21, 0.0, 0.1, 0.0)
        .setBrakeMode(true, true)
        .setLimits(4.9, 13.0, 7.0, 27.5)
        .setDriverProfile(2.0, 2.0, 4.2, 2.0)
        .setPowerProperties(Constants.kVoltage, 65.0, 40.0)
        .setMechanicalProperties(6.75, 150.0 / 7.0, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.05, 0.05, 0.01)
        .setIMU(SwerveIMUs.adis16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Port.kOnboardCS0, CalibrationTime._4s))
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private final SwerveAPI api;

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        api = new SwerveAPI(kConfig);

        api.enableTunables("Swerve");
    }

    @Override
    public void periodic() {
        api.refresh();
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(ForwardPerspective.OPERATOR))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement.
     * @param pose The new blue origin relative pose to apply to the pose estimator.
     */
    public Command resetPose(Supplier<Pose2d> pose) {
        return commandBuilder("Swerve.resetPose()")
            .onInitialize(() -> api.resetPose(pose.get()))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> angular) {
        return commandBuilder("Swerve.drive()").onExecute(() ->
            api.applyDriverInput(x.get(), y.get(), angular.get(), ForwardPerspective.OPERATOR, true, true)
        );
    }
}
