package org.team340.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    public static final double TELEMETRY_PERIOD = 0.020;
    public static final double POWER_USAGE_PERIOD = 0.020;
    public static final double VOLTAGE = 12.0;
    public static final double FIELD_LENGTH = 16.541;
    public static final double FIELD_WIDTH = 8.211;

    /**
     * Driver and co-driver controller constants.
     */
    public static final class ControllerConstants {

        public static final double DRIVE_EXP = 1.0;
        public static final double DRIVE_MULTIPLIER = 0.9;
        public static final double DRIVE_MULTIPLIER_MODIFIED = 0.975;

        public static final double DRIVE_ROT_EXP = 2.0;
        public static final double DRIVE_ROT_MULTIPLIER = 0.4;

        public static final Controller2Config DRIVER = new Controller2Config()
            .setLabel("Driver")
            .setPort(0)
            .setJoystickDeadband(0.15)
            .setJoystickThreshold(0.7)
            .setTriggerDeadband(0.1)
            .setTriggerThreshold(0.1);

        public static final Controller2Config CO_DRIVER = new Controller2Config()
            .setLabel("CoDriver")
            .setPort(1)
            .setJoystickDeadband(0.1)
            .setJoystickThreshold(0.7)
            .setTriggerDeadband(0.1)
            .setTriggerThreshold(0.1);
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

        public static final int PIVOT_LOWER_LIMIT = 7;
        public static final int SHOOTER_NOTE_DETECTOR = 8;
        public static final int INTAKE_NOTE_DETECTOR = 9;

        public static final int LIGHTS = 9;
    }

    public static final class ClimberConstants {

        // Limits
        public static final double MIN_POS = 0.0;
        public static final double MAX_POS = 120.0;

        // Speeds
        public static final double CLIMBING_SPEED = 1.0;

        // Misc
        public static final double CLOSED_LOOP_ERR = Math.toRadians(5.0);
        public static final double BALANCE_COMPENSATION = 0.1;

        // Hardware Configs
        public static final class Configs {

            // Relative Encoder Conversion Factor
            private static final double REL_ENC_FACTOR = 1.0;

            public static final SparkFlexConfig MOTOR = new SparkFlexConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(40)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(true)
                .setClosedLoopRampRate(0.2)
                .setOpenLoopRampRate(0.2);

            public static final RelativeEncoderConfig ENCODER = new RelativeEncoderConfig()
                .setPositionConversionFactor(REL_ENC_FACTOR)
                .setVelocityConversionFactor(REL_ENC_FACTOR / 60);

            public static final SparkLimitSwitchConfig LIMIT = new SparkLimitSwitchConfig().enableLimitSwitch(false);
        }
    }

    public static final class FeederConstants {

        // Speeds
        public static final double RECEIVE_SPEED = 0.5;
        public static final double INTAKE_HUMAN_SPEED = -0.1;
        public static final double SEAT_SPEED = 0.05;
        public static final double REVERSE_SEAT_SPEED = -0.04;
        public static final double SHOOT_SPEED = 1.0;
        public static final double BARF_FORWARD_SPEED = -0.5;
        public static final double POOP_SPEED = -1.0;
        public static final double BARF_BACKWARD_SPEED = 0.5;

        // Positions
        public static final double SEAT_POSITION = 2.357;

        // Misc
        public static final double CLOSED_LOOP_ERR = 0.125;
        public static final double SHOOT_DELAY = 0.3;

        // Hardware Configs
        public static final class Configs {

            // Relative Encoder Conversion Factor
            private static final double REL_ENC_FACTOR = 1.0;

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
                .setPositionConversionFactor(REL_ENC_FACTOR)
                .setVelocityConversionFactor(REL_ENC_FACTOR / 60);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig().setPID(0.15, 0.0, 0.0);
        }
    }

    public static final class IntakeConstants {

        // Limits
        public static final double MIN_POS = 0.0;
        public static final double MAX_POS = Math.toRadians(135.0);

        // Speeds
        public static final double INTAKE_SPEED = 0.9;
        public static final double AMP_HANDOFF_SPEED = -0.25;
        public static final double AMP_UPPER_SPEED = -0.6;
        public static final double AMP_LOWER_SPEED = -0.075;
        public static final double BARF_SPEED = -0.5;
        public static final double POOP_SPEED = -1.0;
        public static final double OVERRIDE_INTAKE_SPEED = 0.25;

        // Positions
        public static final double DOWN_POSITION = Math.toRadians(0.0);
        public static final double SAFE_POSITION = Math.toRadians(30.0);
        public static final double POOP_POSITION = Math.toRadians(25.0);
        public static final double RETRACT_POSITION = Math.toRadians(65.0);
        public static final double UPRIGHT_POSITION = Math.toRadians(90.0);
        public static final double BARF_POSITION = Math.toRadians(10.0);
        public static final double AMP_POSITION = Math.toRadians(56.0);
        public static final double PID_INACTIVE_POSITION = Math.toRadians(2.0);

        // Misc
        public static final double CLOSED_LOOP_ERR = Math.toRadians(5.0);
        public static final double ALLOWABLE_DIFFERENCE = Math.toRadians(15.0);

        // Arm Hardware Configs
        public static final class ArmConfigs {

            // Relative Encoder Conversion Factor
            private static final double REL_ENC_FACTOR = Math2.TWO_PI;

            private static final SparkFlexConfig MOTOR_BASE = new SparkFlexConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(60, 30)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(true)
                .setClosedLoopRampRate(0.35)
                .setOpenLoopRampRate(0.35);

            public static final SparkFlexConfig LEFT_MOTOR = MOTOR_BASE.clone();
            public static final SparkFlexConfig RIGHT_MOTOR = MOTOR_BASE
                .clone()
                .follow(ExternalFollower.kFollowerSpark, RobotMap.INTAKE_ARM_LEFT_MOTOR, false);

            public static final SparkAbsoluteEncoderConfig ENCODER = new SparkAbsoluteEncoderConfig()
                .setPositionConversionFactor(REL_ENC_FACTOR)
                .setVelocityConversionFactor(REL_ENC_FACTOR / 60.0)
                .setInverted(true)
                .setZeroOffset(4.843);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig()
                .setPID(1.0, 0.0015, 0.02)
                .setIZone(Math.toRadians(5.25))
                .setPositionPIDWrappingEnabled(true)
                .setPositionPIDWrappingInputLimits(0.0, REL_ENC_FACTOR);
        }

        // Roller Hardware Configs
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

    public static final class LightsConstants {

        public static final int LENGTH = 22;
    }

    public static final class PivotConstants {

        // Limits
        public static final double MIN_POS = 0.0;
        public static final double MAX_POS = Math.toRadians(89.0);

        // Speeds
        public static final double HOMING_SPEED = -0.2;

        // Positions
        public static final double BARF_FORWARD_POSITION = Math.toRadians(0.0);
        public static final double DOWN_POSITION = Math.toRadians(2.0);
        public static final double ROCK_SKIP_POSITION = Math.toRadians(0.25);
        public static final double MARY_POPPINS_POSITION = Math.toRadians(45.0);
        public static final double AMP_HANDOFF_POSITION = Math.toRadians(0.0);
        public static final double FIX_DEADZONE_POSITION = Math.toRadians(57.5);
        public static final double INTAKE_SAFE_POSITION = Math.toRadians(60.0);

        // Misc
        public static final double CLOSED_LOOP_ERR = Math.toRadians(0.015);
        public static final double AT_LIMIT_SPEED_ALLOWANCE = -0.075;

        // Hardware Configs
        public static final class Configs {

            // Relative Encoder Conversion Factor
            private static final double REL_ENC_FACTOR = Math.toRadians(1.02432);

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
                .setPositionConversionFactor(REL_ENC_FACTOR)
                .setVelocityConversionFactor(REL_ENC_FACTOR / 60);
        }

        // Shooting position lookup table
        public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();

        static {
            DISTANCE_MAP.put(1.31, Math.toRadians(59.01));
            DISTANCE_MAP.put(1.53, Math.toRadians(52.90));
            DISTANCE_MAP.put(1.81, Math.toRadians(46.82));
            DISTANCE_MAP.put(2.09, Math.toRadians(42.10));
            DISTANCE_MAP.put(2.09, Math.toRadians(42.10));
            DISTANCE_MAP.put(2.71, Math.toRadians(37.76));
            DISTANCE_MAP.put(2.83, Math.toRadians(35.65));
            DISTANCE_MAP.put(3.05, Math.toRadians(33.54));
            DISTANCE_MAP.put(3.16, Math.toRadians(33.38));
            DISTANCE_MAP.put(3.45, Math.toRadians(31.25));
            DISTANCE_MAP.put(4.09, Math.toRadians(27.74));
            DISTANCE_MAP.put(4.40, Math.toRadians(26.46));
            DISTANCE_MAP.put(4.44, Math.toRadians(26.39));
            DISTANCE_MAP.put(4.60, Math.toRadians(25.72));
            DISTANCE_MAP.put(4.84, Math.toRadians(23.27));
            DISTANCE_MAP.put(5.07, Math.toRadians(23.15));
            DISTANCE_MAP.put(5.38, Math.toRadians(21.75));
            DISTANCE_MAP.put(5.63, Math.toRadians(21.12));
            DISTANCE_MAP.put(5.94, Math.toRadians(20.39));
            DISTANCE_MAP.put(6.38, Math.toRadians(19.61));
            DISTANCE_MAP.put(6.55, Math.toRadians(20.7));
            DISTANCE_MAP.put(6.87, Math.toRadians(19.76));
            DISTANCE_MAP.put(7.55, Math.toRadians(18.0));
            DISTANCE_MAP.put(8.88, Math.toRadians(16.9));
            DISTANCE_MAP.put(9.71, Math.toRadians(16.88));
            // DISTANCE_MAP.put(1.44, Math.toRadians(55.0));
            // DISTANCE_MAP.put(1.61, Math.toRadians(52.0));
            // DISTANCE_MAP.put(1.99, Math.toRadians(48.0));
            // DISTANCE_MAP.put(2.34, Math.toRadians(42.0));
            // DISTANCE_MAP.put(2.87, Math.toRadians(35.0));
            // DISTANCE_MAP.put(3.34, Math.toRadians(32.0));
            // DISTANCE_MAP.put(3.92, Math.toRadians(29.0));
            // DISTANCE_MAP.put(4.28, Math.toRadians(27.0));
            // DISTANCE_MAP.put(5.32, Math.toRadians(23.0));
            // DISTANCE_MAP.put(5.82, Math.toRadians(21.75));
            // DISTANCE_MAP.put(6.55, Math.toRadians(20.7));
            // DISTANCE_MAP.put(6.87, Math.toRadians(19.76));
            // DISTANCE_MAP.put(7.55, Math.toRadians(18.0));
            // DISTANCE_MAP.put(8.88, Math.toRadians(16.9));
            // DISTANCE_MAP.put(9.71, Math.toRadians(16.88));
        }
    }

    public static final class ShooterConstants {

        // Speeds
        public static final double RAMP_SPEED = 0.95;
        public static final double INTAKE_HUMAN_SPEED = -0.175;
        public static final double FORWARD_BARF_SPEED = -0.5;
        public static final double BACKWARD_BARF_SPEED = 0.5;
        public static final double ROCK_SKIP_SPEED = 0.6;
        public static final double MARY_POPPINS_SPEED = 0.24;

        // Misc
        public static final double CLOSED_LOOP_ERR = 40.0;
        public static final double PID_ACTIVE_RANGE = 750.0;
        public static final double RIGHT_PERCENT_OF_LEFT = 0.55;

        // Hardware Configs
        public static final class Configs {

            // Relative Encoder Conversion Factor
            private static final double REL_ENC_CONVERSION = 2.0;

            private static final SparkFlexConfig MOTOR_BASE = new SparkFlexConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(50, 35)
                .setIdleMode(IdleMode.kCoast)
                .setClosedLoopRampRate(0.0)
                .setOpenLoopRampRate(0.0);

            public static final SparkFlexConfig LEFT_MOTOR = MOTOR_BASE.clone().setInverted(true);
            public static final SparkFlexConfig RIGHT_MOTOR = MOTOR_BASE.clone().setInverted(false);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig()
                .setPID(0.00075, 0.0001, 0.0003)
                .setIZone(10.0);

            public static final RelativeEncoderConfig ENCODER = new RelativeEncoderConfig()
                .setPositionConversionFactor(REL_ENC_CONVERSION)
                .setVelocityConversionFactor(REL_ENC_CONVERSION);

            public static final FeedForwardConfig FEED_FORWARD_LEFT = new FeedForwardConfig(
                0.11331 / 60.0,
                0.065448 / 60.0,
                0.076179 / 60.0
            );
            public static final FeedForwardConfig FEED_FORWARD_RIGHT = new FeedForwardConfig(
                0.11331 / 60.0,
                0.064905 / 60.0,
                0.071392 / 60.0
            );
            public static final SysIdRoutine.Config SYSID = new SysIdRoutine.Config();
        }

        // Shooting speed lookup table
        public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();

        static {
            DISTANCE_MAP.put(0.0, 3000.0);
            DISTANCE_MAP.put(6.0, 6500.0);
            DISTANCE_MAP.put(10.0, 7750.0);
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
            .setMovePID(0.069, 0.001, 0.032, 0.075)
            .setMoveFF(0.13312, 2.3443, 0.3159)
            .setTurnPID(0.65, 0.001, 3.0, 0.01)
            .setRampRate(0.03, 0.03)
            .setMotorTypes(SwerveMotor.Type.SPARK_FLEX_BRUSHLESS, SwerveMotor.Type.SPARK_FLEX_BRUSHLESS)
            .setMaxSpeeds(4.95, 11.8)
            .setRatelimits(8.5, 29.75)
            .setTrajectoryConstraints(3.8, 2.4)
            .setPowerProperties(VOLTAGE, 60.0, 40.0)
            .setMechanicalProperties(6.75, 150.0 / 7.0, 3.81)
            .setDiscretizationLookahead(0.040)
            .setOdometryPeriod(PERIOD)
            .setOdometryStd(0.003, 0.003, 0.0012)
            .setVisionStd(0.0, 0.0, 0.0)
            .setSysIdConfig(new SysIdRoutine.Config(Volts.of(1.0).per(Seconds.of(0.4)), Volts.of(7.0), Seconds.of(5.5)))
            .setFieldSize(FIELD_LENGTH, FIELD_WIDTH)
            .addModule(FRONT_LEFT)
            .addModule(BACK_LEFT)
            .addModule(BACK_RIGHT)
            .addModule(FRONT_RIGHT);

        public static final Transform3d FRONT_LEFT_CAMERA = new Transform3d(
            new Translation3d(0.28364, 0.28369, 0.24511),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(45.0))
        );
        public static final Transform3d BACK_LEFT_CAMERA = new Transform3d(
            new Translation3d(-0.29153, 0.26629, 0.24511),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-160.0))
        );
        public static final Transform3d BACK_RIGHT_CAMERA = new Transform3d(
            new Translation3d(-0.29153, -0.26629, 0.24511),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(160.0))
        );
        public static final Transform3d FRONT_RIGHT_CAMERA = new Transform3d(
            new Translation3d(0.28364, -0.28369, 0.24511),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-45.0))
        );

        public static final double VISION_FIELD_MARGIN = 0.5;
        public static final double VISION_Z_MARGIN = 0.75;
        public static final double VISION_STD_XY_SCALE = 0.006;
        public static final double VISION_STD_ROT_SCALE = 0.015;

        public static final PIDConfig TRAJ_XY_PID = new PIDConfig(10.0, 0.0, 0.1, 0.0);
        public static final PIDConfig TRAJ_ROT_PID = new PIDConfig(2.1, 0.0, 0.1, 0.0);
        public static final Constraints TRAJ_ROT_CONSTRAINTS = new Constraints(6.5, 7.0);

        public static final PIDConfig XY_PID = new PIDConfig(3.5, 1.4, 0.2, 0.5);
        public static final PIDConfig ROT_PID = new PIDConfig(4.9, 0.5, 0.2, 0.2);
        public static final Constraints ROT_CONSTRAINTS = new Constraints(8.0, 37.5);

        public static final double NOTE_VELOCITY = 5.0;
        public static final double NORM_FUDGE = 0.0;
        public static final double NORM_FUDGE_MIN = 0.075;
        public static final double SPIN_COMPENSATION = Math.toRadians(-2.0);
        public static final double FACING_SPEAKER_EPSILON = Math.toRadians(5.0);
    }

    public static final class FieldPositions {

        public static final Translation2d BLUE_SPEAKER = new Translation2d(0.03, 5.5479);
        public static final Translation2d RED_SPEAKER = new Translation2d(BLUE_SPEAKER.getX(), FIELD_WIDTH - BLUE_SPEAKER.getY());

        public static final double SPEAKER_HEIGHT = 2.08;
        public static final Pose3d BLUE_SPEAKER_3D = new Pose3d(
            BLUE_SPEAKER.getX(),
            BLUE_SPEAKER.getY(),
            SPEAKER_HEIGHT,
            Math2.ROTATION3D_0
        );
        public static final Pose3d RED_SPEAKER_3D = new Pose3d(RED_SPEAKER.getX(), RED_SPEAKER.getY(), SPEAKER_HEIGHT, Math2.ROTATION3D_0);

        public static final double AMP_X = 1.775;
        public static final Pose2d AMP_APPROACH_BLUE = new Pose2d(AMP_X, 6.98, Math2.ROTATION2D_HALF_PI);
        public static final Pose2d AMP_APPROACH_RED = new Pose2d(
            AMP_X,
            FIELD_WIDTH - AMP_APPROACH_BLUE.getY(),
            Math2.ROTATION2D_NEG_HALF_PI
        );
        public static final Pose2d AMP_SCORE_BLUE = new Pose2d(AMP_X, 7.82, Math2.ROTATION2D_HALF_PI);
        public static final Pose2d AMP_SCORE_RED = new Pose2d(AMP_X, FIELD_WIDTH - AMP_SCORE_BLUE.getY(), Math2.ROTATION2D_NEG_HALF_PI);

        public static final Translation2d STAGE = new Translation2d(4.981067, 4.105783);

        public static final Translation2d FEED_BLUE = new Translation2d(0.0, 7.4);
        public static final Translation2d FEED_RED = new Translation2d(FEED_BLUE.getX(), FIELD_WIDTH - FEED_BLUE.getY());

        public static final double OPPONENT_WING_LINE = 10.66;
        public static final double MIDLINE = FIELD_LENGTH / 2.0;

        public static final double FENDER_SHOT_DISTANCE = 1.4;
    }
}
