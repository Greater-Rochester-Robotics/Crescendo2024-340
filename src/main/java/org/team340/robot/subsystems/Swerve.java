package org.team340.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.epilogue.Logged;
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

@Logged
public class Swerve extends GRRSubsystem {

    private static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(0.288925, 0.288925)
        .setMoveMotor(SwerveMotors.sparkFlex(RobotMap.FL_MOVE, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkFlex(RobotMap.FL_TURN, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkFlexEncoder(0.3802, true));

    private static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(0.288925, -0.288925)
        .setMoveMotor(SwerveMotors.sparkFlex(RobotMap.FR_MOVE, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkFlex(RobotMap.FR_TURN, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkFlexEncoder(0.8859, true));

    private static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-0.288925, 0.288925)
        .setMoveMotor(SwerveMotors.sparkFlex(RobotMap.BL_MOVE, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkFlex(RobotMap.BL_TURN, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkFlexEncoder(0.3182, true));

    private static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-0.288925, -0.288925)
        .setMoveMotor(SwerveMotors.sparkFlex(RobotMap.BR_MOVE, MotorType.kBrushless, true))
        .setTurnMotor(SwerveMotors.sparkFlex(RobotMap.BR_TURN, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.sparkFlexEncoder(0.6675, true));

    public static final SwerveConfig CONFIG = new SwerveConfig()
        .setTimings(Constants.PERIOD, Constants.PERIOD, 0.04)
        .setMovePID(0.0015, 0.0, 0.0, 0.0)
        .setMoveFF(0.05, 0.212)
        .setTurnPID(0.45, 0.0, 0.1, 0.0)
        .setBrakeMode(false, true)
        .setLimits(4.9, 13.0, 7.0, 27.5)
        .setDriverProfile(4.3, 1.0, 4.2, 2.0)
        .setPowerProperties(Constants.VOLTAGE, 80.0, 60.0)
        .setMechanicalProperties(6.75, 150.0 / 7.0, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.003, 0.003, 0.0012)
        .setIMU(SwerveIMUs.adis16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Port.kOnboardCS0, CalibrationTime._4s))
        .setModules(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

    private final SwerveAPI api;

    public Swerve() {
        api = new SwerveAPI(CONFIG);
        api.enableTunables("Swerve");
    }

    @Override
    public void periodic() {
        api.refresh();
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

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(ForwardPerspective.OPERATOR))
            .isFinished(true);
    }
}
