package org.team340.robot;

import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.team340.lib.controller.Controller2Config;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.PIDConfig;
import org.team340.lib.util.config.rev.AbsoluteEncoderConfig;
import org.team340.lib.util.config.rev.SparkFlexConfig;
import org.team340.lib.util.config.rev.SparkMaxConfig;
import org.team340.lib.util.config.rev.SparkPIDControllerConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double PERIOD = 0.020;
    public static final double TELEMETRY_PERIOD = 0.040;
    public static final double POWER_USAGE_PERIOD = 0.020;
    public static final double VOLTAGE = 12.0;
    public static final double FIELD_LENGTH = 16.5417;
    public static final double FIELD_WIDTH = 8.0136;

    /**
     * Driver and co-driver controller constants.
     */
    public static final class ControllerConstants {

        public static final Controller2Config DRIVER = new Controller2Config()
            .setLabel("Driver")
            .setPort(0)
            .setJoystickDeadband(0.1)
            .setJoystickThreshold(0.7)
            .setTriggerDeadband(0.1)
            .setTriggerThreshold(0.1)
            .setLeftProfile("joystickprofiles/DriverLeft.json")
            .setRightProfile("joystickprofiles/DriverRight.json");

        public static final Controller2Config CO_DRIVER = new Controller2Config()
            .setLabel("CoDriver")
            .setPort(1)
            .setJoystickDeadband(0.1)
            .setJoystickThreshold(0.7)
            .setTriggerDeadband(0.1)
            .setTriggerThreshold(0.1)
            .setLeftProfile("joystickprofiles/CoDriverLeft.json")
            .setRightProfile("joystickprofiles/CoDriverRight.json");

        public static final double DRIVE_EXP = 1.0;
        public static final double DRIVE_MULTIPLIER = 0.75;
        public static final double DRIVE_MULTIPLIER_MODIFIED = 0.95;

        public static final double DRIVE_ROT_EXP = 2.0;
        public static final double DRIVE_ROT_MULTIPLIER = 0.4;
    }

    /**
     * Map of hardware device IDs.
     */
    public static final class RobotMap {

        public static final int FRONT_LEFT_MOVE = 2;
        public static final int FRONT_LEFT_TURN = 3;
        public static final int BACK_LEFT_MOVE = 4;
        public static final int BACK_LEFT_TURN = 5;
        public static final int BACK_RIGHT_MOVE = 6;
        public static final int BACK_RIGHT_TURN = 7;
        public static final int FRONT_RIGHT_MOVE = 8;
        public static final int FRONT_RIGHT_TURN = 9;

        public static final int INTAKE_ARM_LEFT_MOTOR = 20;
        public static final int INTAKE_ARM_RIGHT_MOTOR = 21;
        public static final int INTAKE_ROLLER_UPPER_MOTOR = 22;
        public static final int INTAKE_ROLLER_LOWER_MOTOR = 23;

        public static final int SHOOTER_PIVOT_MOTOR = 30;
        public static final int SHOOTER_FEED_MOTOR = 31;
        public static final int SHOOTER_SHOOT_LEFT_MOTOR = 32;
        public static final int SHOOTER_SHOOT_RIGHT_MOTOR = 33;

        public static final int SHOOTER_NOTE_DETECTOR = 0;
        public static final int PIVOT_LOWER_LIMIT = 1;
    }

    public static final class IntakeConstants {

        private static final SparkMaxConfig ARM_MOTOR_BASE_CONFIG = new SparkMaxConfig()
            .clearFaults()
            .restoreFactoryDefaults()
            .enableVoltageCompensation(VOLTAGE)
            .setSmartCurrentLimit(30)
            .setIdleMode(IdleMode.kBrake)
            .setClosedLoopRampRate(1.5)
            .setOpenLoopRampRate(1.5);
        private static final SparkMaxConfig ROLLER_MOTOR_BASE_CONFIG = new SparkMaxConfig()
            .clearFaults()
            .restoreFactoryDefaults()
            .enableVoltageCompensation(VOLTAGE)
            .setSmartCurrentLimit(30)
            .setIdleMode(IdleMode.kCoast)
            .setClosedLoopRampRate(1.5)
            .setOpenLoopRampRate(1.5);

        public static final SparkMaxConfig ARM_LEFT_MOTOR_CONFIG = ARM_MOTOR_BASE_CONFIG.clone().setInverted(false);
        public static final SparkMaxConfig ARM_RIGHT_MOTOR_CONFIG = ARM_MOTOR_BASE_CONFIG
            .clone()
            .follow(ExternalFollower.kFollowerSpark, RobotMap.INTAKE_ARM_LEFT_MOTOR, false);
        public static final SparkMaxConfig ROLLER_UPPER_MOTOR_CONFIG = ROLLER_MOTOR_BASE_CONFIG.clone().setInverted(false);
        public static final SparkMaxConfig ROLLER_LOWER_MOTOR_CONFIG = ROLLER_MOTOR_BASE_CONFIG
            .clone()
            .follow(ExternalFollower.kFollowerSpark, RobotMap.INTAKE_ROLLER_UPPER_MOTOR, true);

        public static final int ARM_WEAK_PID_SLOT = 0;
        public static final int ARM_STRONG_PID_SLOT = 1;

        public static final SparkPIDControllerConfig ARM_MOTOR_PID_CONFIG = new SparkPIDControllerConfig()
            .setPID(0.0, 0.0, 0.0, ARM_WEAK_PID_SLOT)
            .setPID(0.0, 0.0, 0.0, ARM_STRONG_PID_SLOT);

        public static final AbsoluteEncoderConfig ARM_ENCODER_CONFIG = new AbsoluteEncoderConfig()
            .setZeroOffset(0.0)
            .setInverted(false)
            .setPositionConversionFactor(Math2.TWO_PI)
            .setVelocityConversionFactor(Math2.TWO_PI / 60.0);

        public static final double DEPLOY_POSITION = 0.0;
        public static final double DEPLOY_POSITION_TOLERANCE = 0.0;

        public static final double DEPLOY_ROLLER_SPEED = 0.0;
        public static final double SPIT_ROLLER_SPEED = -0.0;
    }

    public static final class ShooterConstants {

        private static final SparkFlexConfig SHOOT_MOTOR_BASE_CONFIG = new SparkFlexConfig()
            .clearFaults()
            .restoreFactoryDefaults()
            .enableVoltageCompensation(VOLTAGE)
            .setSmartCurrentLimit(60, 30)
            .setIdleMode(IdleMode.kCoast)
            .setClosedLoopRampRate(1.5)
            .setOpenLoopRampRate(1.5);

        public static final SparkFlexConfig SHOOT_LEFT_MOTOR_CONFIG = SHOOT_MOTOR_BASE_CONFIG.clone().setInverted(false);
        public static final SparkFlexConfig SHOOT_RIGHT_MOTOR_CONFIG = SHOOT_MOTOR_BASE_CONFIG.clone().setInverted(true);

        public static final SparkMaxConfig FEED_MOTOR_CONFIG = new SparkMaxConfig()
            .clearFaults()
            .restoreFactoryDefaults()
            .enableVoltageCompensation(VOLTAGE)
            .setSmartCurrentLimit(30)
            .setIdleMode(IdleMode.kCoast)
            .setClosedLoopRampRate(1.5)
            .setOpenLoopRampRate(1.5);

        public static final SparkPIDControllerConfig FEED_PID_CONFIG = new SparkPIDControllerConfig().setPID(0.0, 0.0, 0.0, 0);
        public static final SparkPIDControllerConfig SHOOT_PID_CONFIG = new SparkPIDControllerConfig().setPID(0.0, 0.0, 0.0, 0);

        public static final double FEED_INTAKE_SPEED = 0.0;
    }

    /**
     * All distances in inches. All angles in radians.
     */
    public static final class PivotConstants {

        public static final SparkMaxConfig PIVOT_MOTOR_CONFIG = new SparkMaxConfig()
            .clearFaults()
            .restoreFactoryDefaults()
            .enableVoltageCompensation(VOLTAGE)
            .setSmartCurrentLimit(30)
            .setIdleMode(IdleMode.kCoast)
            .setClosedLoopRampRate(1.5)
            .setOpenLoopRampRate(1.5);

        public static final SparkPIDControllerConfig PIVOT_PID_CONFIG = new SparkPIDControllerConfig().setPID(0.0, 0.0, 0.0, 0);

        public static final double PID_MIN_OUTPUT = 0.0;
        public static final double PID_MAX_OUTPUT = 0.0;
        public static final double MIN_VEL = 0.0;
        public static final double MAX_VEL = 0.0;
        public static final double MAX_ACCEL = 0.0;
        public static final double CLOSED_LOOP_ERR = 0.0;
        public static final double REL_ENC_CONVERSION = 0.2;

        public static final double SHOOTER_LENGTH = 0.0;
        public static final double BASE_LENGTH = 0.0;
        public static final double MINIMUM_LENGTH_OF_DART = 0.0;
        public static final double SUM_OF_SQUARES_OF_LENGTHS = SHOOTER_LENGTH * SHOOTER_LENGTH + BASE_LENGTH * BASE_LENGTH;

        public static final double TWICE_THE_PRODUCT_OF_LENGTHS = 2 * SHOOTER_LENGTH * BASE_LENGTH;

        public static final double OFFSET_ANGLE = Math.acos(
            (SUM_OF_SQUARES_OF_LENGTHS - MINIMUM_LENGTH_OF_DART * MINIMUM_LENGTH_OF_DART) / TWICE_THE_PRODUCT_OF_LENGTHS
        );

        public static final double MINIMUM_ANGLE = 0.0;
        public static final double MAXIMUM_ANGLE = 0.0;

        public static final double HOMING_SPEED = 0.0;
    }

    /**
     * Constants for the swerve subsystem.
     */
    public static final class SwerveConstants {

        private static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig()
            .setLabel("Front Left")
            .useSparkAttachedEncoder(0.0, false)
            .setPosition(0.3000, 0.3000)
            .setMoveMotor(RobotMap.FRONT_LEFT_MOVE, true, true)
            .setTurnMotor(RobotMap.FRONT_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig()
            .setLabel("Back Left")
            .useSparkAttachedEncoder(0.0, false)
            .setPosition(-0.3000, 0.3000)
            .setMoveMotor(RobotMap.BACK_LEFT_MOVE, true, true)
            .setTurnMotor(RobotMap.BACK_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig()
            .setLabel("Back Right")
            .useSparkAttachedEncoder(0.0, false)
            .setPosition(-0.3000, -0.3000)
            .setMoveMotor(RobotMap.BACK_RIGHT_MOVE, true, true)
            .setTurnMotor(RobotMap.BACK_RIGHT_TURN, false, true);

        private static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig()
            .setLabel("Front Right")
            .useSparkAttachedEncoder(0.0, false)
            .setPosition(0.3000, -0.3000)
            .setMoveMotor(RobotMap.FRONT_RIGHT_MOVE, true, true)
            .setTurnMotor(RobotMap.FRONT_RIGHT_TURN, false, true);

        public static final SwerveConfig CONFIG = new SwerveConfig()
            .useADIS16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Port.kOnboardCS0, CalibrationTime._4s)
            .setPeriod(PERIOD)
            .setMovePID(0.001, 0.0, 0.0, 0.0)
            .setMoveFF(0.0, 2.84, 0.0)
            .setTurnPID(0.5, 0.0, 15.0, 0.0)
            .setRampRate(0.03, 0.03)
            .setMotorTypes(SwerveMotor.Type.SPARK_FLEX_BRUSHLESS, SwerveMotor.Type.SPARK_FLEX_BRUSHLESS)
            .setMaxSpeeds(5.0, 10.0)
            .setRatelimits(17.5, 30.0)
            .setPowerProperties(VOLTAGE, 60.0, 30.0)
            .setMechanicalProperties(6.5, 7.0, 4.0)
            .setDiscretizationLookahead(0.020)
            .setOdometryPeriod(0.005)
            .setOdometryStd(0.1, 0.1, 0.1)
            .setSysIdConfig(new SysIdRoutine.Config())
            .setFieldSize(FIELD_LENGTH, FIELD_WIDTH)
            .addModule(FRONT_LEFT)
            .addModule(BACK_LEFT)
            .addModule(BACK_RIGHT)
            .addModule(FRONT_RIGHT);

        public static final PIDConfig ROT_PID = new PIDConfig(7.0, 0.0, 0.5, 0.0);
        public static final Constraints ROT_CONSTRAINTS = new Constraints(6.0, 12.5);
    }
}
