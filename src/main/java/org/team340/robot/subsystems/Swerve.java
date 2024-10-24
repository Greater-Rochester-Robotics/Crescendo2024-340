package org.team340.robot.subsystems;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team340.lib.dashboard.Tunable;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveAPI.ForwardPerspective;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.CommandBuilder;
import org.team340.lib.util.DummySubsystem;
import org.team340.lib.util.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants;
import org.team340.robot.Constants.FieldConstants;
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
        .setBrakeMode(false, true)
        .setLimits(4.9, 13.0, 7.0, 27.5)
        .setDriverProfile(4.3, 1.0, 4.2, 2.0)
        .setPowerProperties(Constants.kVoltage, 65.0, 40.0)
        .setMechanicalProperties(6.75, 150.0 / 7.0, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.05, 0.05, 0.01)
        .setIMU(SwerveIMUs.adis16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Port.kOnboardCS0, CalibrationTime._4s))
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private static final Transform3d kBackLeftCamera = new Transform3d(
        new Translation3d(-0.29153, 0.26629, 0.24511),
        new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-160.0))
    );
    private static final Transform3d kBackRightCamera = new Transform3d(
        new Translation3d(-0.29153, -0.26629, 0.24511),
        new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(160.0))
    );

    private static final double kTrajKp = 7.5;
    private static final double kTrajKi = 0.0;
    private static final double kTrajKd = 0.0;
    private static final double kTrajIZone = 0.0;

    private static final double kTrajAngularKp = 4.75;
    private static final double kTrajAngularKi = 0.0;
    private static final double kTrajAngularKd = 0.0;
    private static final double kTrajAngularIZone = 0.0;

    private static final double kAngularKp = 6.25;
    private static final double kAngularKi = 0.5;
    private static final double kAngularKd = 0.0;
    private static final double kAngularIZone = 0.8;
    private static final Constraints kAngularConstraints = new Constraints(9.5, 24.0);

    private static final Tunable<Double> kDistanceFudge = Tunable.doubleValue("Swerve/kDistanceFudge", 0.4);
    private static final Tunable<Double> kNoteVelocity = Tunable.doubleValue("Swerve/kNoteVelocity", 5.6);
    private static final Tunable<Double> kNormFudge = Tunable.doubleValue("Swerve/kNormFudge", 0.49);
    private static final Tunable<Double> kNormFudgeMin = Tunable.doubleValue("Swerve/kNormFudgeMin", 0.49);
    private static final Tunable<Double> kStrafeFudge = Tunable.doubleValue("Swerve/kStrafeFudge", 0.85);
    private static final Tunable<Double> kSpinCompensation = Tunable.doubleValue("Swerve/kSpinCompensation", 0.035);

    private final SwerveAPI api;

    private final PIDController trajPIDx;
    private final PIDController trajPIDy;
    private final PIDController trajAngularPID;
    private final ProfiledPIDController angularPID;

    private final AutoFactory autoFactory;

    private final AprilTagFieldLayout aprilTags;
    private final PhotonPoseEstimator[] poseEstimators;
    private final List<Pose2d> measurements = new ArrayList<>();
    private final List<Pose3d> targets = new ArrayList<>();

    private final Tunable<Double> speakerXFudge = Tunable.doubleValue("Swerve/speakerXFudge", 0.0);
    private final Tunable<Double> speakerYFudge = Tunable.doubleValue("Swerve/speakerYFudge", 0.0);

    private final Subsystem trajAimSpeakerMutex = new DummySubsystem();
    private boolean trajAimSpeaker = false;
    private Pose2d trajNext = null;
    private Pose2d trajLast = null;

    private Translation2d speaker = new Translation2d();
    private double speakerDistance = 0.0;
    private double speakerAngle = 0.0;

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        api = new SwerveAPI(kConfig);

        trajPIDx = new PIDController(kTrajKp, kTrajKi, kTrajKd);
        trajPIDy = new PIDController(kTrajKp, kTrajKi, kTrajKd);
        trajPIDx.setIZone(kTrajIZone);
        trajPIDy.setIZone(kTrajIZone);

        trajAngularPID = new PIDController(kTrajAngularKp, kTrajAngularKi, kTrajAngularKd);
        trajAngularPID.setIZone(kTrajAngularIZone);
        trajAngularPID.enableContinuousInput(-Math.PI, Math.PI);

        angularPID = new ProfiledPIDController(kAngularKp, kAngularKi, kAngularKd, kAngularConstraints);
        angularPID.setIZone(kAngularIZone);
        angularPID.enableContinuousInput(-Math.PI, Math.PI);

        autoFactory = Choreo.createAutoFactory(
            this,
            () -> api.state.pose,
            (Pose2d pose, SwerveSample sample) -> {
                trajLast = trajNext;
                trajNext = sample.getPose();

                double omega;
                if (trajAimSpeaker) {
                    omega = angularPID.calculate(api.state.pose.getRotation().getRadians(), getSpeakerAngle());
                } else {
                    omega = trajAngularPID.calculate(pose.getRotation().getRadians(), sample.heading) + sample.omega;
                }

                api.applySpeeds(
                    new ChassisSpeeds(
                        trajPIDx.calculate(pose.getX(), sample.x) + sample.vx,
                        trajPIDy.calculate(pose.getY(), sample.y) + sample.vy,
                        omega
                    ),
                    ForwardPerspective.BLUE_ALLIANCE,
                    true,
                    false
                );
            },
            Alliance::isRed,
            new AutoBindings(),
            (traj, running) -> {
                if (!running) {
                    trajLast = null;
                    trajNext = trajLast;
                }
            }
        );

        aprilTags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        aprilTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        poseEstimators = new PhotonPoseEstimator[] {
            new PhotonPoseEstimator(
                aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new PhotonCamera("backleft"),
                kBackLeftCamera
            ),
            new PhotonPoseEstimator(
                aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new PhotonCamera("backright"),
                kBackRightCamera
            )
        };

        api.enableTunables("Swerve");
        Tunable.pidController("Swerve/trajPID", trajPIDx);
        Tunable.pidController("Swerve/trajPID", trajPIDy);
        Tunable.pidController("Swerve/trajAngularPID", trajAngularPID);
        Tunable.pidController("Swerve/angularPID", angularPID);
    }

    @Override
    public void periodic() {
        api.refresh();

        measurements.clear();
        targets.clear();

        for (int i = 0; i < poseEstimators.length; i++) {
            var estimate = poseEstimators[i].update();
            if (estimate.isPresent()) {
                double weightedSum = 0.0;
                for (var target : estimate.get().targetsUsed) {
                    int id = target.getFiducialId();
                    Optional<Pose3d> tagPose = aprilTags.getTagPose(id);
                    if (tagPose.isEmpty()) continue;
                    targets.add(tagPose.get());
                    double distance = api.state.pose
                        .getTranslation()
                        .getDistance(tagPose.get().getTranslation().toTranslation2d());
                    boolean important = id == 3 || id == 4 || id == 7 || id == 8;
                    weightedSum += distance * (important ? 0.65 : 1.0);
                }

                Pose3d estimatedPose = estimate.get().estimatedPose;
                int tagCount = estimate.get().targetsUsed.size();
                double stdScale = Math.pow(weightedSum / tagCount, 2.0) / tagCount;
                double xyStd = 0.2 * stdScale;
                double angStd = 0.3 * stdScale;

                api.addVisionMeasurement(
                    estimatedPose.toPose2d(),
                    estimate.get().timestampSeconds,
                    VecBuilder.fill(xyStd, xyStd, angStd)
                );

                measurements.add(estimatedPose.toPose2d());
            }
        }

        Translation2d realSpeaker = Alliance.isBlue() ? FieldConstants.kBlueSpeaker : FieldConstants.kRedSpeaker;
        Translation2d robot = api.state.pose.getTranslation();
        ChassisSpeeds robotVel = api.state.speeds;
        double distance = robot.getDistance(realSpeaker);
        double normFactor = Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond) < kNormFudgeMin.get()
            ? 0.0
            : Math.abs(
                MathUtil.angleModulus(
                    robot.minus(realSpeaker).getAngle().getRadians() -
                    Math.atan2(robotVel.vyMetersPerSecond, robotVel.vxMetersPerSecond)
                ) /
                Math.PI
            );

        double x = realSpeaker.getX() + speakerXFudge.get();
        double y = realSpeaker.getY() + speakerYFudge.get();

        if (!trajAimSpeaker) {
            x -=
            (robotVel.vxMetersPerSecond * (distance / kNoteVelocity.get()) * (1.0 - (kNormFudge.get() * normFactor)));
            y -= (robotVel.vyMetersPerSecond * (distance / kNoteVelocity.get()) * kStrafeFudge.get());
        }

        speaker = new Translation2d(x, y);
        speakerDistance = robot.getDistance(speaker) + kDistanceFudge.get();
        speakerAngle = MathUtil.angleModulus(
            speaker.minus(robot).getAngle().getRadians() + Math.PI + kSpinCompensation.get()
        );
    }

    public AutoFactory autoFactory() {
        return autoFactory;
    }

    /**
     * Gets a field-relative position for the shot to the speaker
     * the robot should take, adjusted for the robot's movement.
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getSpeaker() {
        return speaker;
    }

    /**
     * Returns the distance from the speaker in meters, adjusted for the robot's movement.
     */
    public double getSpeakerDistance() {
        return speakerDistance;
    }

    /**
     * Gets the angle for the robot to face to score in the speaker,
     * in radians. Compensates for note drift caused by spin.
     */
    private double getSpeakerAngle() {
        return speakerAngle;
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
     * Signals to the trajectory controller to aim at the speaker. This command does not
     * require the swerve subsystem, so it can be ran in parallel to a drive command.
     * This command will not end on its own. When it is cancelled, the trajectory
     * controller will be signaled to no longer aim at the speaker.
     */
    public Command trajAimSpeaker() {
        return new CommandBuilder("Swerve.trajAimSpeaker()", trajAimSpeakerMutex)
            .onInitialize(() -> {
                trajAimSpeaker = true;
                angularPID.reset(api.state.pose.getRotation().getRadians(), api.state.speeds.omegaRadiansPerSecond);
            })
            .onEnd(() -> trajAimSpeaker = false)
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

    /**
     * Drives the robot using driver input, while
     * forcing the robot to face the speaker.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command driveSpeaker(Supplier<Double> x, Supplier<Double> y) {
        return commandBuilder("Swerve.driveSpeaker()")
            .onInitialize(() ->
                angularPID.reset(api.state.pose.getRotation().getRadians(), api.state.speeds.omegaRadiansPerSecond)
            )
            .onExecute(() -> {
                double angularVel = angularPID.calculate(api.state.pose.getRotation().getRadians(), getSpeakerAngle());
                api.applyDriverXY(x.get(), y.get(), angularVel, ForwardPerspective.OPERATOR, true, true);
            });
    }

    /**
     * Drives the robot using driver input, while
     * forcing the robot to face the amp.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    public Command driveAmp(Supplier<Double> x, Supplier<Double> y) {
        return commandBuilder("Swerve.driveAmp()")
            .onInitialize(() ->
                angularPID.reset(api.state.pose.getRotation().getRadians(), api.state.speeds.omegaRadiansPerSecond)
            )
            .onExecute(() -> {
                double angularVel = angularPID.calculate(api.state.pose.getRotation().getRadians(), -Math2.HALF_PI);
                api.applyDriverXY(x.get(), y.get(), angularVel, ForwardPerspective.OPERATOR, true, true);
            });
    }

    /**
     * Allows the driver to keep driving, but forces the robot to face the feed location.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     */
    public Command driveFeed(Supplier<Double> x, Supplier<Double> y) {
        return commandBuilder("Swerve.driveFeed()")
            .onInitialize(() ->
                angularPID.reset(api.state.pose.getRotation().getRadians(), api.state.speeds.omegaRadiansPerSecond)
            )
            .onExecute(() -> {
                Translation2d feedPoint = Alliance.isBlue() ? FieldConstants.kBlueFeed : FieldConstants.kRedFeed;
                double faceFeedAngle = MathUtil.angleModulus(
                    feedPoint.minus(api.state.pose.getTranslation()).getAngle().getRadians() + Math.PI
                );

                double angularVel = angularPID.calculate(api.state.pose.getRotation().getRadians(), faceFeedAngle);
                api.applyDriverXY(x.get(), y.get(), angularVel, ForwardPerspective.OPERATOR, true, true);
            });
    }

    public Command lockWheels() {
        return commandBuilder("Swerve.lockWheels()").onExecute(() -> api.applyLockedWheels());
    }
}
