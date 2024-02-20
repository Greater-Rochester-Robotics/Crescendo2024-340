package org.team340.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.AccelStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import org.team340.lib.util.config.FeedForwardConfig;
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

    public static final double NOTE_VELOCITY = 20.0;

    public static final Translation2d BLUE_SPEAKER = new Translation2d(0.0241, 5.65);
    public static final Translation2d RED_SPEAKER = new Translation2d(BLUE_SPEAKER.getX(), FIELD_WIDTH - BLUE_SPEAKER.getY());
    public static final Translation2d STAGE = new Translation2d(4.981067, 4.105783);
    public static final double AMP_X = 1.9526;

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
            .setJoystickDeadband(0.15)
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

        public static final double MINIMUM_ANGLE = 0.0;
        public static final double MAXIMUM_ANGLE = Math.toRadians(135.0);

        public static final double MINIMUM_PID_ANGLE = Math.toRadians(2.0);
        public static final double CLOSED_LOOP_ERROR = Math.toRadians(5.0);

        public static final double DOWN_POSITION = 0.0;
        public static final double SCORE_AMP_POSITION = Math.toRadians(55.0);
        public static final double SAFE_POSITION = Math.toRadians(30.0);
        public static final double RETRACT_POSITION = Math.toRadians(65.0);
        public static final double UPRIGHT_POSITION = Math.toRadians(90.0);
        public static final double SPIT_POSITION = Math.toRadians(10.0);

        public static final double SCORE_AMP_ROLLER_SPEED_UPPER = -0.5;
        public static final double SCORE_AMP_ROLLER_SPEED_LOWER = -0.1;
        public static final double INTAKE_ROLLER_SPEED = 0.9;
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
                .setClosedLoopRampRate(0.15)
                .setOpenLoopRampRate(0.15);
            public static final SparkFlexConfig LEFT_MOTOR = MOTOR_BASE.clone().setInverted(true);
            public static final SparkFlexConfig RIGHT_MOTOR = MOTOR_BASE
                .clone()
                .follow(ExternalFollower.kFollowerSpark, RobotMap.INTAKE_ARM_LEFT_MOTOR, false);
            public static final SparkAbsoluteEncoderConfig ENCODER = new SparkAbsoluteEncoderConfig()
                .setInverted(true)
                .setPositionConversionFactor(Math2.TWO_PI)
                .setVelocityConversionFactor(Math2.TWO_PI / 60.0)
                .setZeroOffset(4.8304510);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig()
                .setPID(0.9, 0.0015, 0.075) //0.425, 0.0013, 0.32)
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

        public static final double SPEED_TOLERANCE = 40.0;

        public static final double LEFT_SPIT_SPEED_BACK = 0.5;
        public static final double RIGHT_SPIT_SPEED_BACK = 0.5;
        public static final double LEFT_SPIT_SPEED_FRONT = -0.25;
        public static final double RIGHT_SPIT_SPEED_FRONT = -0.25;
        public static final double LEFT_INTAKE_HUMAN_SPEED = -0.3;
        public static final double RIGHT_INTAKE_HUMAN_SPEED = -0.3;

        public static final double RIGHT_TO_LEFT_RATIO = 0.55;
        public static final double PID_RANGE = 750.0;
        public static final double RAMP_SPEED = 0.95;

        public static final double REL_ENC_CONVERSION = 2.0; // gear ratio

        public static final class Configs {

            private static final SparkFlexConfig MOTOR_BASE = new SparkFlexConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(60, 40)
                .setIdleMode(IdleMode.kCoast)
                .setClosedLoopRampRate(0.0)
                .setOpenLoopRampRate(0.0);
            public static final SparkFlexConfig LEFT_MOTOR = MOTOR_BASE.clone().setInverted(true);
            public static final SparkFlexConfig RIGHT_MOTOR = MOTOR_BASE.clone().setInverted(false);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig().setPID(0.001, 0.0, 0.0);

            public static final FeedForwardConfig FEED_FORWARD = new FeedForwardConfig(0.11331 / 60.0, 0.060404 / 60.0, 0.064897 / 60.0);

            public static final RelativeEncoderConfig ENCODER = new RelativeEncoderConfig()
                .setPositionConversionFactor(REL_ENC_CONVERSION)
                .setVelocityConversionFactor(REL_ENC_CONVERSION);

            public static final SysIdRoutine.Config SYSID = new SysIdRoutine.Config();
        }

        public static final InterpolatingDoubleTreeMap DISTANCE_TO_SPEED_MAP = new InterpolatingDoubleTreeMap();

        static {
            DISTANCE_TO_SPEED_MAP.put(0.0, 3000.0);
            DISTANCE_TO_SPEED_MAP.put(3.0, 4250.0);
            DISTANCE_TO_SPEED_MAP.put(6.0, 6000.0);
            DISTANCE_TO_SPEED_MAP.put(8.0, 6750.0);
            DISTANCE_TO_SPEED_MAP.put(10.0, 7500.0);
        }
    }

    public static final class FeederConstants {

        public static final double SHOOT_DELAY = 0.5;

        public static final double INTAKE_SPEED = 0.5;
        public static final double IN_SLOW_SPEED = 0.05;
        public static final double POSITION_OFFSET = 2.357;
        public static final double CLOSED_LOOP_ERR = 0.125;
        public static final double SHOOT_SPEED = 1.0;
        public static final double INTAKE_HUMAN_SPEED = -0.25;
        public static final double SPIT_SPEED_FRONT = -0.5;
        public static final double SPIT_SPEED_BACK = 0.5;

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
        public static final double SAFE_FOR_INTAKE_ANGLE = Math.toRadians(60.0);
        public static final double OPTIMAL_RECEIVE_NOTE_ANGLE = Math.toRadians(0.0);

        public static final double HOMING_SPEED = -0.2;
        public static final double AT_LIMIT_SPEED_ALLOWANCE = -0.025;

        public static final double REL_ENC_CONVERSION = Math.toRadians(1.02432);

        public static final class Configs {

            public static final SparkFlexConfig MOTOR = new SparkFlexConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(60, 30)
                .setIdleMode(IdleMode.kCoast)
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

        public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();

        public static final double SPIT_ANGLE = 0.0;

        static {
            DISTANCE_TO_ANGLE_MAP.put(1.37, Math.toRadians(59.0));
            DISTANCE_TO_ANGLE_MAP.put(1.65, Math.toRadians(52.0));
            DISTANCE_TO_ANGLE_MAP.put(1.97, Math.toRadians(45.0));
            DISTANCE_TO_ANGLE_MAP.put(2.29, Math.toRadians(39.0));
            DISTANCE_TO_ANGLE_MAP.put(2.72, Math.toRadians(35.0));
            DISTANCE_TO_ANGLE_MAP.put(3.17, Math.toRadians(32.0));
            DISTANCE_TO_ANGLE_MAP.put(4.07, Math.toRadians(29.0));
            DISTANCE_TO_ANGLE_MAP.put(4.95, Math.toRadians(25.0));
            DISTANCE_TO_ANGLE_MAP.put(6.46, Math.toRadians(21.0));
            DISTANCE_TO_ANGLE_MAP.put(7.38, Math.toRadians(18.75));
            DISTANCE_TO_ANGLE_MAP.put(8.28, Math.toRadians(17.25));
        }
    }

    public static final class ClimberConstants {

        public static final double REL_ENC_CONVERSION = 1; // (125 * 12 * 0.25); // 1 / (gear ratio * sprocket teeth * inches/tooth)
        public static final double CLOSED_LOOP_ERR = 0.125;
        public static final double ZEROING_SPEED = 0.5;

        public static final double MAX_POS = 120.0;
        public static final double MIN_POS = 0.0;

        public static final class Configs {

            public static final SparkMaxConfig MOTOR = new SparkMaxConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(30)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(true)
                .setClosedLoopRampRate(1.5)
                .setOpenLoopRampRate(1.5);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig()
                .setPID(1.0, 0.0, 0.0, 0)
                .setOutputRange(-.5, .5)
                .setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0)
                .setSmartMotionMaxVelocity(0, 0)
                .setSmartMotionMinOutputVelocity(0, 0)
                .setSmartMotionMaxAccel(0, 0)
                .setSmartMotionAllowedClosedLoopError(CLOSED_LOOP_ERR * REL_ENC_CONVERSION, 0);

            public static final FeedForwardConfig FF = new FeedForwardConfig(0.0, 0.0, 0.0);

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
            .setMovePID(0.0125, 0.001, 0.005, 0.075)
            .setMoveFF(0.13312, 2.3443, 0.3159)
            .setTurnPID(0.65, 0.001, 3.0, 0.01)
            .setRampRate(0.03, 0.03)
            .setMotorTypes(SwerveMotor.Type.SPARK_FLEX_BRUSHLESS, SwerveMotor.Type.SPARK_FLEX_BRUSHLESS)
            .setMaxSpeeds(4.95, 11.8)
            .setRatelimits(7.9, 28.75)
            .setPowerProperties(VOLTAGE, 60.0, 40.0)
            .setMechanicalProperties(6.75, 150.0 / 7.0, 4.0)
            .setDiscretizationLookahead(0.020)
            .setOdometryPeriod(0.020)
            .setOdometryStd(0.03, 0.03, 0.075)
            .setVisionStd(0.1, 0.1, 0.125)
            .setSysIdConfig(new SysIdRoutine.Config(Volts.of(1.0).per(Seconds.of(0.4)), Volts.of(7.0), Seconds.of(5.5)))
            .setFieldSize(FIELD_LENGTH, FIELD_WIDTH)
            .addModule(FRONT_LEFT)
            .addModule(BACK_LEFT)
            .addModule(BACK_RIGHT)
            .addModule(FRONT_RIGHT);

        public static final PIDConfig AUTO_XY_PID = new PIDConfig(0.0, 0.0, 0.0, 0.0);
        public static final PIDConfig AUTO_ROT_PID = new PIDConfig(0.0, 0.0, 0.0, 0.0);

        public static final PIDConfig XY_PID = new PIDConfig(2.0, 0.0, 0.1, 0.0);
        public static final PIDConfig ROT_PID = new PIDConfig(5.5, 0.0, 0.2, 0.0);
        public static final Constraints XY_CONSTRAINTS = new Constraints(3.0, 6.0);
        public static final Constraints ROT_CONSTRAINTS = new Constraints(7.0, 10.0);

        public static final double POSE_XY_ERROR = 0.025;
        public static final double POSE_ROT_ERROR = Math.toRadians(5.0);

        public static final Transform3d BACK_LEFT_CAMERA = new Transform3d(
            new Translation3d(-0.29153, 0.26629, 0.24511),
            new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(-160.0))
        );
        public static final Transform3d BACK_RIGHT_CAMERA = new Transform3d(
            new Translation3d(-0.29153, -0.26629, 0.24511),
            new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(160.0))
        );

        public static final Pose2d AMP_APPROACH_BLUE = new Pose2d(AMP_X, 6.5, new Rotation2d(Math2.HALF_PI));
        public static final Pose2d AMP_SCORE_BLUE = new Pose2d(AMP_X, 5.5, new Rotation2d(Math2.HALF_PI));
        public static final Pose2d AMP_APPROACH_RED = new Pose2d(
            AMP_X,
            FIELD_WIDTH - AMP_APPROACH_BLUE.getY(),
            new Rotation2d(-Math2.HALF_PI)
        );
        public static final Pose2d AMP_SCORE_RED = new Pose2d(AMP_X, FIELD_WIDTH - AMP_SCORE_BLUE.getY(), new Rotation2d(-Math2.HALF_PI));
    }
}
