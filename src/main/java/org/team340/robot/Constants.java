package org.team340.robot;

import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.AccelStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.team340.lib.controller.Controller2Config;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.PIDConfig;
import org.team340.lib.util.config.rev.RelativeEncoderConfig;
import org.team340.lib.util.config.rev.SparkAbsoluteEncoderConfig;
import org.team340.lib.util.config.rev.SparkFlexConfig;
import org.team340.lib.util.config.rev.SparkLimitSwitchConfig;
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

    public static final Translation2d RED_TARGET = new Translation2d();
    public static final Translation2d BLUE_TARGET = new Translation2d();

    /**
     * Driver and co-driver controller constants.
     */
    public static final class ControllerConstants {

        public static final double DRIVE_EXP = 1.0;
        public static final double DRIVE_MULTIPLIER = 0.85;
        public static final double DRIVE_MULTIPLIER_MODIFIED = 0.95;

        public static final double DRIVE_ROT_EXP = 2.0;
        public static final double DRIVE_ROT_MULTIPLIER = 0.4;

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
        public static final int SHOOTER_FEEDER_MOTOR = 31;
        public static final int SHOOTER_SHOOT_LEFT_MOTOR = 32;
        public static final int SHOOTER_SHOOT_RIGHT_MOTOR = 33;

        public static final int CLIMBER_LEFT_MOTOR = 40;
        public static final int CLIMBER_RIGHT_MOTOR = 41;

        public static final int CLIMBER_LEFT_LIMIT = 2;
        public static final int CLIMBER_RIGHT_LIMIT = 3;
        public static final int PIVOT_LOWER_LIMIT = 7;
        public static final int SHOOTER_NOTE_DETECTOR = 8;
        public static final int INTAKE_NOTE_DETECTOR = 9;
    }

    public static final class IntakeConstants {

        public static final double MAXIMUM_ANGLE = Math.toRadians(135.0);
        public static final double MINIMUM_ANGLE = 0.0;
        public static final double MINIMUM_PID_ANGLE = 0.0;

        public static final double CLOSED_LOOP_ERROR = Math.toRadians(5.0);

        public static final double SCORE_AMP_POSITION = Math.toRadians(55.0);
        public static final double DEPLOY_POSITION = 0.0;
        public static final double STRAIGHT_UP_POSITION = Math2.HALF_PI;
        public static final double SAFE_POSITION = Math.toRadians(65.0);

        public static final double SCORE_AMP_ROLLER_SPEED = -0.80;
        public static final double INTAKE_ROLLER_SPEED = 0.40;
        public static final double SPIT_ROLLER_SPEED = -0.5;
        public static final double FROM_SHOOTER_ROLLER_SPEED = -0.25;

        public static final double AMP_SCORING_TIMEOUT = 2.0;

        public static final class ArmConfigs {

            private static final SparkFlexConfig MOTOR_BASE = new SparkFlexConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(60, 30)
                .setIdleMode(IdleMode.kBrake)
                .setClosedLoopRampRate(0.3)
                .setOpenLoopRampRate(0.3);
            public static final SparkFlexConfig LEFT_MOTOR = MOTOR_BASE.clone().setInverted(true);
            public static final SparkFlexConfig RIGHT_MOTOR = MOTOR_BASE
                .clone()
                .follow(ExternalFollower.kFollowerSpark, RobotMap.INTAKE_ARM_LEFT_MOTOR, false);
            public static final SparkAbsoluteEncoderConfig ENCODER = new SparkAbsoluteEncoderConfig()
                .setInverted(true)
                .setPositionConversionFactor(Math2.TWO_PI)
                .setVelocityConversionFactor(Math2.TWO_PI / 60.0)
                .setZeroOffset(2.832); // TODO: put in this angle before running

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig()
                .setPID(0.425, 0.0013, 0.32)
                .setIZone(Math.toRadians(5.25))
                .setPositionPIDWrappingEnabled(true)
                .setPositionPIDWrappingInputLimits(0.0, Math2.TWO_PI);
        }

        public static final class RollerConfigs {

            public static final SparkMaxConfig MOTOR = new SparkMaxConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(30)
                .setIdleMode(IdleMode.kCoast)
                .setInverted(true)
                .setClosedLoopRampRate(0.0)
                .setOpenLoopRampRate(0.0);
        }
    }

    public static final class ShooterConstants {

        public static final double SPEED_TOLERANCE = 300.0;

        public static final double LEFT_SPIT_SPEED = -0.25;
        public static final double RIGHT_SPIT_SPEED = -0.25;

        public static final double RIGHT_TO_LEFT_RATIO = 0.55;

        public static final double REL_ENC_CONVERSION = 2.0; // gear ratio

        public static final class Configs {

            private static final SparkFlexConfig MOTOR_BASE = new SparkFlexConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(60, 30)
                .setIdleMode(IdleMode.kCoast)
                .setClosedLoopRampRate(0.0)
                .setOpenLoopRampRate(0.0);
            public static final SparkFlexConfig LEFT_MOTOR = MOTOR_BASE.clone().setInverted(true);
            public static final SparkFlexConfig RIGHT_MOTOR = MOTOR_BASE.clone().setInverted(false);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig().setPIDF(65.0, 0.0, 15.0, 1.0 / 9200.0, 0);

            public static final RelativeEncoderConfig ENCODER = new RelativeEncoderConfig()
                .setPositionConversionFactor(REL_ENC_CONVERSION)
                .setVelocityConversionFactor(REL_ENC_CONVERSION);
        }

        private static final InterpolatingDoubleTreeMap DISTANCE_TO_SPEED_MAP = new InterpolatingDoubleTreeMap();

        static { // TODO: find these
            DISTANCE_TO_SPEED_MAP.put(0.0, 0.1);
            DISTANCE_TO_SPEED_MAP.put(1.499, 0.25);
            DISTANCE_TO_SPEED_MAP.put(1.5, 0.8);
            DISTANCE_TO_SPEED_MAP.put(25.0, 0.9);
        }

        public static double interpolateSpeed(Pose2d robotPosition, Translation2d targetPosition) {
            // Find the distance between the two
            double distance = robotPosition.getTranslation().getDistance(targetPosition);
            return DISTANCE_TO_SPEED_MAP.get(distance);
        }

        public static double interpolateSpeed(Pose2d robotPosition) {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
            return interpolateSpeed(robotPosition, alliance == Alliance.Red ? RED_TARGET : BLUE_TARGET);
        }
    }

    public static final class FeederConstants {

        public static final double SHOOT_DELAY = 0.5;

        public static final double INTAKE_SPEED = 1.0;
        public static final double IN_SLOW_SPEED = 0.05;
        public static final double POSITION_OFFSET = 2.357;
        public static final double CLOSED_LOOP_ERR = 0.125;
        public static final double SHOOT_SPEED = 1.0;
        public static final double SPIT_SPEED = -0.5;

        public static final double REL_ENC_CONVERSION = 1.0; // 1 / ((30 / 64) * 1.4 * Math.PI); // 1 / (gear ratio * roller diameter * pi)

        public static final class Configs {

            public static final SparkMaxConfig MOTOR = new SparkMaxConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(30)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(true)
                .setClosedLoopRampRate(0.0)
                .setOpenLoopRampRate(0.0);

            public static final RelativeEncoderConfig ENCODER = new RelativeEncoderConfig()
                .setPositionConversionFactor(REL_ENC_CONVERSION)
                .setVelocityConversionFactor(REL_ENC_CONVERSION / 60);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig().setPID(0.15, 0.0, 0.0);
        }
    }

    public static final class PivotConstants {

        public static final double CLOSED_LOOP_ERR = Math.toRadians(0.015);

        public static final double MINIMUM_ANGLE = 0.0;
        public static final double MAXIMUM_ANGLE = Math.toRadians(89.0);
        public static final double SAFE_FOR_INTAKE_ANGLE = Math.toRadians(65.0);
        public static final double OPTIMAL_RECEIVE_NOTE_ANGLE = Math.toRadians(0.0);

        public static final double HOMING_SPEED = -0.2;

        public static final double REL_ENC_CONVERSION = Math.toRadians(1.02432); // 1 / (25 * 1.1 * Math.PI / 7.568); // 1 / (gear ratio * circ output pinion / radius of arc)

        public static final class Configs {

            public static final SparkFlexConfig MOTOR = new SparkFlexConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(60, 30)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(true)
                .setClosedLoopRampRate(0.25)
                .setOpenLoopRampRate(0.25);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig()
                .setPID(2.3, 0.004, 2.7)
                .setIZone(Math.toRadians(2.0));

            public static final RelativeEncoderConfig ENCODER = new RelativeEncoderConfig()
                .setPositionConversionFactor(REL_ENC_CONVERSION)
                .setVelocityConversionFactor(REL_ENC_CONVERSION / 60);
        }

        private static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();

        static { // TODO: find these
            DISTANCE_TO_ANGLE_MAP.put(0.0, 0.0);
            DISTANCE_TO_ANGLE_MAP.put(5.6, 1.2);
        }

        public static double interpolateAngle(Pose2d robotPosition, Translation2d targetPosition) {
            // Find the distance between the two
            double distance = robotPosition.getTranslation().getDistance(targetPosition);
            return DISTANCE_TO_ANGLE_MAP.get(distance);
        }

        public static double interpolateAngle(Pose2d robotPosition) {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
            return interpolateAngle(robotPosition, alliance == Alliance.Red ? RED_TARGET : BLUE_TARGET);
        }
    }

    public static final class ClimberConstants {

        public static final double REL_ENC_CONVERSION = 1 / (125 * 12 * 0.25); // 1 / (gear ratio * sprocket teeth * inches/tooth)
        public static final double CLOSED_LOOP_ERR = 0.125;
        public static final double ZEROING_SPEED = 0.25;

        public static final double MAX_POS = 1.0;
        public static final double MIN_POS = 0.0;

        public static final class Configs {

            public static final SparkMaxConfig MOTOR = new SparkMaxConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(30)
                .setIdleMode(IdleMode.kBrake)
                .setClosedLoopRampRate(1.5)
                .setOpenLoopRampRate(1.5);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig()
                .setPID(0.0, 0.0, 0.0, 0)
                .setOutputRange(-.5, .5)
                .setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0)
                .setSmartMotionMaxVelocity(0, 0)
                .setSmartMotionMinOutputVelocity(0, 0)
                .setSmartMotionMaxAccel(0, 0)
                .setSmartMotionAllowedClosedLoopError(CLOSED_LOOP_ERR * REL_ENC_CONVERSION, 0);

            public static final RelativeEncoderConfig ENCODER = new RelativeEncoderConfig()
                .setPositionConversionFactor(REL_ENC_CONVERSION)
                .setVelocityConversionFactor(REL_ENC_CONVERSION / 60);

            public static final SparkLimitSwitchConfig LIMIT = new SparkLimitSwitchConfig().enableLimitSwitch(true);
        }
    }

    public static final class SwerveConstants {

        private static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig()
            .setLabel("Front Left")
            .useSparkAttachedEncoder(2.389, true)
            .setPosition(0.288925, 0.288925)
            .setMoveMotor(RobotMap.FRONT_LEFT_MOVE, true, true)
            .setTurnMotor(RobotMap.FRONT_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig()
            .setLabel("Back Left")
            .useSparkAttachedEncoder(1.999, true)
            .setPosition(-0.288925, 0.288925)
            .setMoveMotor(RobotMap.BACK_LEFT_MOVE, true, true)
            .setTurnMotor(RobotMap.BACK_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig()
            .setLabel("Back Right")
            .useSparkAttachedEncoder(4.194, true)
            .setPosition(-0.288925, -0.288925)
            .setMoveMotor(RobotMap.BACK_RIGHT_MOVE, true, true)
            .setTurnMotor(RobotMap.BACK_RIGHT_TURN, false, true);

        private static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig()
            .setLabel("Front Right")
            .useSparkAttachedEncoder(5.566, true)
            .setPosition(0.288925, -0.288925)
            .setMoveMotor(RobotMap.FRONT_RIGHT_MOVE, true, true)
            .setTurnMotor(RobotMap.FRONT_RIGHT_TURN, false, true);

        public static final SwerveConfig CONFIG = new SwerveConfig()
            .useADIS16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Port.kOnboardCS0, CalibrationTime._4s)
            .setPeriod(PERIOD)
            .setMovePID(0.001, 0.0, 0.0, 0.0)
            .setMoveFF(0.13312, 2.3443, 0.3159)
            .setTurnPID(0.5, 0.0, 3.0, 0.0)
            .setRampRate(0.03, 0.03)
            .setMotorTypes(SwerveMotor.Type.SPARK_FLEX_BRUSHLESS, SwerveMotor.Type.SPARK_FLEX_BRUSHLESS)
            .setMaxSpeeds(4.9, 11.8)
            .setRatelimits(7.9, 8.75)
            .setPowerProperties(VOLTAGE, 60.0, 40.0)
            .setMechanicalProperties(6.75, 150.0 / 7.0, 4.0)
            .setDiscretizationLookahead(0.020)
            .setOdometryPeriod(0.02)
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
