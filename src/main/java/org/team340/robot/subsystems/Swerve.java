package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team340.lib.swerve.SwerveBase;
import org.team340.lib.swerve.util.SwerveVisualizer;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants;
import org.team340.robot.Constants.FieldPositions;
import org.team340.robot.Constants.SwerveConstants;

/**\[]
 * The swerve subsystem.
 */
public class Swerve extends SwerveBase {

    private final PIDController xPIDTraj = SwerveConstants.TRAJ_XY_PID.pidController();
    private final PIDController yPIDTraj = SwerveConstants.TRAJ_XY_PID.pidController();
    private final PIDController rotPIDTraj = SwerveConstants.TRAJ_ROT_PID.pidController();
    private final ProfiledPIDController rotPIDTrajProfiled = SwerveConstants.TRAJ_ROT_PID.profiledPIDController(
        SwerveConstants.TRAJ_ROT_CONSTRAINTS
    );

    private final HolonomicDriveController trajectoryController = new HolonomicDriveController(xPIDTraj, yPIDTraj, rotPIDTrajProfiled);

    private final PIDController xPID = SwerveConstants.XY_PID.pidController();
    private final PIDController yPID = SwerveConstants.XY_PID.pidController();
    private final ProfiledPIDController rotPID = SwerveConstants.ROT_PID.profiledPIDController(SwerveConstants.ROT_CONSTRAINTS);

    private final AprilTagFieldLayout blueAprilTags;
    private final AprilTagFieldLayout redAprilTags;
    private final PhotonPoseEstimator[] photonPoseEstimators;

    private double[] speaker = new double[0];
    private double tunableNoteVelocity = SwerveConstants.NOTE_VELOCITY;
    private double tunableNormFudge = SwerveConstants.NORM_FUDGE;

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        super("Swerve Drive", SwerveConstants.CONFIG);
        rotPIDTraj.enableContinuousInput(-Math.PI, Math.PI);
        rotPIDTrajProfiled.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.enableContinuousInput(-Math.PI, Math.PI);

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
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("speakerX", () -> getSpeakerPosition().getX(), null);
        builder.addDoubleProperty("speakerY", () -> getSpeakerPosition().getY(), null);
        builder.addDoubleProperty("speakerDistance", this::getSpeakerDistance, null);
        builder.addBooleanProperty("inOpponentWing", this::inOpponentWing, null);
        builder.addDoubleArrayProperty("speaker", () -> speaker, null);
        builder.addDoubleProperty("tunableNoteVelocity", () -> tunableNoteVelocity, velocity -> tunableNoteVelocity = velocity);
        builder.addDoubleProperty("tunableNormFudge", () -> tunableNormFudge, fudge -> tunableNormFudge = fudge);
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
     * Returns {@code true} if the robot is in the opponent's wing.
     */
    public boolean inOpponentWing() {
        return getPosition().getX() > FieldPositions.OPPONENT_WING_LINE;
    }

    /**
     * Returns the distance from the speaker in meters, adjusted for the robot's movement.
     */
    public double getSpeakerDistance() {
        return getPosition().getTranslation().getDistance(getSpeakerPosition());
    }

    /**
     * Returns the distance to the amp in meters.
     */
    public double getAmpDistance() {
        return getPosition()
            .getTranslation()
            .getDistance((Alliance.isBlue() ? FieldPositions.AMP_SCORE_BLUE : FieldPositions.AMP_SCORE_RED).getTranslation());
    }

    /**
     * Returns the distance to the center of the stage in meters.
     */
    public double getStageDistance() {
        return getPosition().getTranslation().getDistance(FieldPositions.STAGE);
    }

    /**
     * Gets a field-relative position for the shot to the speaker
     * the robot should take, adjusted for the robot's movement.
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getSpeakerPosition() {
        Translation2d goalPose = Alliance.isBlue() ? FieldPositions.BLUE_SPEAKER : FieldPositions.RED_SPEAKER;
        Translation2d robotPos = getPosition().getTranslation();
        ChassisSpeeds robotVel = getVelocity(true);
        double distance = robotPos.getDistance(goalPose);
        double normFactor = Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond) < SwerveConstants.NORM_FUDGE_MIN
            ? 0.0
            : Math.abs(
                MathUtil.angleModulus(
                    robotPos.minus(goalPose).getAngle().getRadians() - Math.atan2(robotVel.vyMetersPerSecond, robotVel.vxMetersPerSecond)
                ) /
                Math.PI
            );
        // if (Math.random() > 0.9) System.out.println((1.0 - (tunableNormFudge * normFactor))); lol
        double x =
            goalPose.getX() - (robotVel.vxMetersPerSecond * (distance / tunableNoteVelocity) * (1.0 - (tunableNormFudge * normFactor)));
        double y = goalPose.getY() - (robotVel.vyMetersPerSecond * (distance / tunableNoteVelocity));
        return new Translation2d(x, y);
    }

    /**
     * Gets the angle for the robot to face to score in the speaker,
     * in radians. Compensates for note drift caused by spin.
     */
    private double getSpeakerAngle() {
        Translation2d speakerPosition = getSpeakerPosition();
        double speakerDistance = getSpeakerDistance();
        Translation2d robotPoint = getPosition().getTranslation();
        Rotation2d shotRot = speakerPosition.minus(robotPoint).getAngle();

        Translation2d targetPosition = new Translation2d(
            speakerPosition.getX() + shotRot.getSin() * SwerveConstants.SPIN_COMPENSATION_X * speakerDistance,
            speakerPosition.getY() +
            shotRot.getCos() *
            SwerveConstants.SPIN_COMPENSATION_Y *
            speakerDistance *
            (Alliance.isBlue() ? 1.0 : -1.0)
        );

        speaker =
            SwerveVisualizer.pose3d(
                new Pose3d(targetPosition.getX(), targetPosition.getY(), FieldPositions.SPEAKER_HEIGHT, Math2.ROTATION3D_0)
            );
        return MathUtil.angleModulus(targetPosition.minus(robotPoint).getAngle().getRadians() + Math.PI);
    }

    /**
     * Returns the robot's roll in radians.
     */
    public double getRoll() {
        return imu.getRoll().getRadians();
    }

    /**
     * Zeroes the IMU to a specified yaw.
     */
    public Command zeroIMU(Rotation2d yaw) {
        return runOnce(() -> imu.setZero(yaw)).withName("swerve.zero(" + yaw.toString() + ")");
    }

    /**
     * Dumps the current state of odometry and resets to 0.
     */
    public Command dumpOdometry() {
        return commandBuilder("swerve.dumpOdometry()").onInitialize(() -> resetOdometry(new Pose2d())).isFinished(true);
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

    /**
     * Faces the robot towards the speaker.
     */
    public Command driveSpeaker() {
        return driveSpeaker(() -> 0.0, () -> 0.0);
    }

    /**
     * Allows the driver to keep driving, but forces the robot to face the speaker.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     */
    public Command driveSpeaker(Supplier<Double> x, Supplier<Double> y) {
        return commandBuilder("swerve.driveSpeaker()")
            .onInitialize(() -> rotPID.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> driveAngle(x.get(), y.get(), getSpeakerAngle(), rotPID, false));
    }

    /**
     * Drives to the amp.
     * If PID is being used, the command will not end.
     * If a trajectory is being used, the command ends after the trajectory is completed.
     * @param useTrajectory If a trajectory should be used to translate to the amp. Otherwise, PID is used,
     */
    public Command driveAmp(boolean useTrajectory) {
        return either(
            useTrajectory
                ? defer(() -> followTrajectory(generateTrajectory(FieldPositions.AMP_APPROACH_BLUE, FieldPositions.AMP_SCORE_BLUE)))
                : pidTo(FieldPositions.AMP_SCORE_BLUE),
            useTrajectory
                ? defer(() -> followTrajectory(generateTrajectory(FieldPositions.AMP_APPROACH_RED, FieldPositions.AMP_SCORE_RED)))
                : pidTo(FieldPositions.AMP_SCORE_RED),
            Alliance::isBlue
        )
            .withName("swerve.driveAmp(" + useTrajectory + ")");
    }

    /**
     * Allows the driver to keep driving, but forces the robot to face the stage.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     */
    public Command driveStage(Supplier<Double> x, Supplier<Double> y) {
        return commandBuilder("swerve.alignWithStage")
            .onInitialize(() -> rotPID.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> {
                double stageAngle = FieldPositions.STAGE.minus(getPosition().getTranslation()).getAngle().getRadians();
                double faceStageAngle;
                if (stageAngle >= 0.0 && stageAngle <= Math2.TWO_THIRD_PI) {
                    faceStageAngle = Math2.THIRD_PI;
                } else if (stageAngle >= -Math2.TWO_THIRD_PI && stageAngle <= 0.0) {
                    faceStageAngle = -Math2.THIRD_PI;
                } else {
                    faceStageAngle = -Math.PI;
                }

                driveAngle(x.get(), y.get(), faceStageAngle, rotPID, false);
            });
    }

    /**
     * Moves to a specified pose using PID. Does not end.
     * @param pose The pose to translate to.
     */
    public Command pidTo(Pose2d pose) {
        return commandBuilder("swerve.pidTo(" + pose.toString() + ")")
            .onInitialize(() -> {
                rotPID.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond);
                xPID.reset();
                yPID.reset();
            })
            .onExecute(() -> driveToPose(pose, xPID, yPID, rotPID, false))
            .onEnd(this::stop);
    }

    /**
     * Follows a trajectory.
     * @param traj The trajectory to follow.
     */
    public Command followTrajectory(ChoreoTrajectory traj) {
        return followTrajectory(traj, -1.0, false);
    }

    /**
     * Follows a trajectory.
     * @param traj The trajectory to follow.
     * @param targetTime Time in seconds after the path starts to start targeting the speaker.
     */
    public Command followTrajectory(ChoreoTrajectory traj, double targetTime) {
        return followTrajectory(traj, targetTime, false);
    }

    /**
     * Follows a trajectory.
     * @param traj The trajectory to follow.
     * @param resetOdometry If the odometry should be reset to the first pose in the trajectory.
     */
    public Command followTrajectory(ChoreoTrajectory traj, boolean resetOdometry) {
        return followTrajectory(traj, -1.0, resetOdometry);
    }

    /**
     * Follows a trajectory.
     * @param traj The trajectory to follow.
     * @param targetTime Time in seconds after the path starts to start targeting the speaker.
     * @param resetOdometry If the odometry should be reset to the first pose in the trajectory.
     */
    public Command followTrajectory(ChoreoTrajectory traj, double targetTime, boolean resetOdometry) {
        BiConsumer<Boolean, Pose2d> state = visualizer.addTrajectory(traj.getPoses());

        return Choreo
            .choreoSwerveCommand(
                traj,
                this::getPosition,
                targetTime,
                this::getSpeakerAngle,
                rotPID,
                xPIDTraj,
                yPIDTraj,
                rotPIDTraj,
                speeds -> driveSpeeds(speeds, false, false),
                targetPose -> state.accept(true, targetPose),
                Alliance::isRed,
                this
            )
            .beforeStarting(() -> {
                if (resetOdometry) {
                    Pose2d initialPose = Alliance.isBlue() ? traj.getInitialPose() : traj.getInitialState().flipped().getPose();
                    zeroIMU(initialPose.getRotation());
                    resetOdometry(initialPose);
                }

                rotPID.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond);
                xPIDTraj.reset();
                yPIDTraj.reset();
                rotPIDTraj.reset();
            })
            .finallyDo(() -> state.accept(false, Math2.POSE2D_0))
            .withName("swerve.followTrajectory()");
    }

    /**
     * Follows a trajectory.
     * @param traj The trajectory to follow.
     */
    public Command followTrajectory(Trajectory traj) {
        Rotation2d endRotation = traj.getStates().get(traj.getStates().size() - 1).poseMeters.getRotation();
        Timer timer = new Timer();

        BiConsumer<Boolean, Pose2d> state = visualizer.addTrajectory(
            traj.getStates().stream().map(s -> new Pose2d(s.poseMeters.getX(), s.poseMeters.getY(), endRotation)).toArray(Pose2d[]::new)
        );

        return commandBuilder("swerve.followTrajectory()")
            .onInitialize(() -> {
                rotPIDTrajProfiled.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond);
                xPIDTraj.reset();
                yPIDTraj.reset();
                timer.restart();
            })
            .onExecute(() -> {
                State goal = traj.sample(timer.get());
                ChassisSpeeds speeds = trajectoryController.calculate(getPosition(), goal, endRotation);
                driveSpeeds(speeds, false, false);
                state.accept(true, goal.poseMeters);
            })
            .isFinished(() -> timer.hasElapsed(traj.getTotalTimeSeconds()))
            .onEnd(() -> {
                state.accept(false, Math2.POSE2D_0);
                timer.stop();
                stop();
            });
    }

    /**
     * Runs a SysId quasistatic test.
     * @param direction The direction to run the test in.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Runs a SysId dynamic test.
     * @param direction The direction to run the test in.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
