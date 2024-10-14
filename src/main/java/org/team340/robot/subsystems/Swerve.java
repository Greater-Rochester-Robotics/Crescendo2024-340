package org.team340.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
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
import org.team340.robot.tagslam.GtsamInterface;
import org.team340.robot.tagslam.TagDetection;

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

    private static final SwerveConfig CONFIG = new SwerveConfig()
        .setTimings(Constants.PERIOD, Constants.PERIOD, 0.04)
        .setMovePID(0.0015, 0.0, 0.0, 0.0)
        .setMoveFF(0.05, 0.212)
        .setTurnPID(0.45, 0.0, 0.1, 0.0)
        .setBrakeMode(false, true)
        .setLimits(4.9, 13.0, 7.0, 27.5)
        .setDriverProfile(4.3, 1.0, 4.2, 2.0)
        .setPowerProperties(Constants.VOLTAGE, 80.0, 60.0)
        .setMechanicalProperties(6.75, 150.0 / 7.0, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.05)
        .setIMU(SwerveIMUs.adis16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Port.kOnboardCS0, CalibrationTime._4s))
        .setModules(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

    private static final Transform3d BACK_LEFT_CAMERA = new Transform3d(
        new Translation3d(-0.29153, 0.26629, 0.24511),
        new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-160.0))
    );

    private static final Transform3d BACK_RIGHT_CAMERA = new Transform3d(
        new Translation3d(-0.29153, -0.26629, 0.24511),
        new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(160.0))
    );

    private final SwerveAPI api;

    private final AprilTagFieldLayout aprilTags;
    private final GtsamInterface gtsamInterface;
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] poseEstimators;
    private final PhotonPipelineResult[] lastResults;

    private final List<Pose2d> measurements = new ArrayList<>();
    private final List<Pose3d> targets = new ArrayList<>();
    private Pose2d gtsamPose = new Pose2d();

    public Swerve() {
        api = new SwerveAPI(CONFIG);
        api.enableTunables("Swerve");

        aprilTags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        aprilTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        gtsamInterface = new GtsamInterface(List.of("backleft", "backright"));
        cameras = new PhotonCamera[] { new PhotonCamera("backleft"), new PhotonCamera("backright") };
        poseEstimators = new PhotonPoseEstimator[] {
            new PhotonPoseEstimator(aprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras[0], BACK_LEFT_CAMERA),
            new PhotonPoseEstimator(aprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras[1], BACK_RIGHT_CAMERA)
        };
        lastResults = new PhotonPipelineResult[cameras.length];
        Arrays.fill(lastResults, new PhotonPipelineResult());
    }

    @Override
    public void periodic() {
        long loopStart = WPIUtilJNI.now();
        long tagDetTime = loopStart - 10000; // YIPPEE

        api.refresh();

        measurements.clear();
        targets.clear();

        Pose3d guess = null;
        int mostTargets = 0;
        for (int i = 0; i < cameras.length; i++) {
            var camera = cameras[i];
            var poseEstimator = poseEstimators[i];
            var results = camera.getLatestResult();
            var last = lastResults[i];

            if (results.getTimestampSeconds() != last.getTimestampSeconds()) {
                List<TagDetection> dets = new ArrayList<>();
                lastResults[i] = results;
                for (var result : results.getTargets()) {
                    dets.add(new TagDetection(result.getFiducialId(), result.getDetectedCorners()));
                }

                gtsamInterface.setCamIntrinsics(camera.getName(), camera.getCameraMatrix(), camera.getDistCoeffs());
                gtsamInterface.sendVisionUpdate(
                    camera.getName(),
                    tagDetTime,
                    dets,
                    poseEstimator.getRobotToCameraTransform()
                );

                var estimate = poseEstimators[i].update(results, camera.getCameraMatrix(), camera.getDistCoeffs());
                if (estimate.isPresent()) {
                    double sum = 0.0;
                    for (var target : estimate.get().targetsUsed) {
                        Optional<Pose3d> tagPose = aprilTags.getTagPose(target.getFiducialId());
                        if (tagPose.isEmpty()) continue;
                        targets.add(tagPose.get());
                        sum +=
                        api.state.pose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());
                    }

                    Pose3d estimatedPose = estimate.get().estimatedPose;
                    int tagCount = estimate.get().targetsUsed.size();
                    double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
                    double xyStd = 0.1 * stdScale;
                    double angStd = 0.15 * stdScale;

                    api.addVisionMeasurement(
                        estimatedPose.toPose2d(),
                        estimate.get().timestampSeconds,
                        VecBuilder.fill(xyStd, xyStd, angStd)
                    );

                    measurements.add(estimatedPose.toPose2d());

                    if (tagCount > mostTargets) {
                        mostTargets = tagCount;
                        guess = new Pose3d(estimatedPose.toPose2d());
                    }
                }
            }
        }

        Twist2d twist = api.state.twist;
        gtsamInterface.sendOdomUpdate(loopStart, new Twist3d(twist.dx, twist.dy, 0, 0, 0, twist.dtheta), guess);

        gtsamPose = gtsamInterface.getLatencyCompensatedPoseEstimate().toPose2d();
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
            .isFinished(true)
            .ignoringDisable(true);
    }
}
