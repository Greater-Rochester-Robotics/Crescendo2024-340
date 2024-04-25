package org.team340.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team340.lib.swerve.SwerveBase;
import org.team340.lib.util.Alliance;
import org.team340.robot.Constants;
import org.team340.robot.Constants.SwerveConstants;

/**
 * The swerve subsystem.
 */
public class Swerve extends SwerveBase {

    private final AprilTagFieldLayout blueAprilTags;
    private final AprilTagFieldLayout redAprilTags;
    private final PhotonPoseEstimator[] photonPoseEstimators;

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        super("Swerve Drive", SwerveConstants.CONFIG);
        blueAprilTags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        redAprilTags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        blueAprilTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        redAprilTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);

        photonPoseEstimators =
            new PhotonPoseEstimator[] {
                new PhotonPoseEstimator(
                    blueAprilTags,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    new PhotonCamera("FrontLeft"),
                    SwerveConstants.FRONT_LEFT_CAMERA
                ),
                new PhotonPoseEstimator(
                    blueAprilTags,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    new PhotonCamera("BackLeft"),
                    SwerveConstants.BACK_LEFT_CAMERA
                ),
                new PhotonPoseEstimator(
                    blueAprilTags,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    new PhotonCamera("BackRight"),
                    SwerveConstants.BACK_RIGHT_CAMERA
                ),
                new PhotonPoseEstimator(
                    blueAprilTags,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    new PhotonCamera("FrontRight"),
                    SwerveConstants.FRONT_RIGHT_CAMERA
                ),
            };
    }

    @Override
    public void periodic() {
        updateOdometry(poseEstimator -> {
            List<Pose2d> measurements = new ArrayList<>();
            List<Pose3d> targets = new ArrayList<>();

            Pose2d currentPose = getPosition();
            for (int i = 0; i < photonPoseEstimators.length; i++) {
                Optional<EstimatedRobotPose> pose = photonPoseEstimators[i].update();
                if (pose.isPresent()) {
                    Pose3d raw = pose.get().estimatedPose;
                    Pose3d pose3d = Alliance.isBlue()
                        ? raw
                        : new Pose3d(
                            Constants.FIELD_LENGTH - raw.getX(),
                            Constants.FIELD_WIDTH - raw.getY(),
                            raw.getZ(),
                            raw.getRotation().minus(new Rotation3d(0.0, 0.0, Math.PI))
                        );
                    Pose2d pose2d = pose3d.toPose2d();
                    if (
                        pose3d.getX() >= -SwerveConstants.VISION_FIELD_MARGIN &&
                        pose3d.getX() <= Constants.FIELD_LENGTH + SwerveConstants.VISION_FIELD_MARGIN &&
                        pose3d.getY() >= -SwerveConstants.VISION_FIELD_MARGIN &&
                        pose3d.getY() <= Constants.FIELD_WIDTH + SwerveConstants.VISION_FIELD_MARGIN &&
                        pose3d.getZ() >= -SwerveConstants.VISION_Z_MARGIN &&
                        pose3d.getZ() <= SwerveConstants.VISION_Z_MARGIN
                    ) {
                        double sum = 0.0;
                        for (PhotonTrackedTarget target : pose.get().targetsUsed) {
                            Optional<Pose3d> tagPose =
                                (Alliance.isBlue() ? blueAprilTags : redAprilTags).getTagPose(target.getFiducialId());
                            if (tagPose.isEmpty()) continue;
                            targets.add(tagPose.get());
                            sum += currentPose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());
                        }

                        int tagCount = pose.get().targetsUsed.size();
                        double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
                        double xyStd = SwerveConstants.VISION_STD_XY_SCALE * stdScale;
                        double rotStd = SwerveConstants.VISION_STD_ROT_SCALE * stdScale;

                        poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds, VecBuilder.fill(xyStd, xyStd, rotStd));
                        measurements.add(pose2d);
                        continue;
                    }
                }
            }

            visualizer.updateVision(measurements.stream().toArray(Pose2d[]::new), targets.stream().toArray(Pose3d[]::new));
        });
    }

    /**
     * Zeroes the IMU to a specified yaw.
     */
    public Command zeroIMU(Rotation2d yaw) {
        return runOnce(() -> imu.setZero(yaw)).withName("swerve.zero(" + yaw.toString() + ")");
    }

    /**
     * Drives the robot.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param rot The desired rotational speed from {@code -1.0} to {@code 1.0}.
     * @param fieldRelative If the robot should drive field relative.
     */
    public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot, boolean fieldRelative) {
        return commandBuilder("swerve.drive()").onExecute(() -> drive(x.get(), y.get(), rot.get(), fieldRelative));
    }
}
