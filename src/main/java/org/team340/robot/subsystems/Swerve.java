package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team340.lib.commands.CommandBuilder;
import org.team340.lib.swerve.SwerveBase;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants;
import org.team340.robot.Constants.SwerveConstants;

/**\[]
 * The swerve subsystem.
 */
public class Swerve extends SwerveBase {

    private final PIDController xPIDAuto = new PIDController(
        SwerveConstants.AUTO_XY_PID.p(),
        SwerveConstants.AUTO_XY_PID.i(),
        SwerveConstants.AUTO_XY_PID.d()
    );
    private final PIDController yPIDAuto = new PIDController(
        SwerveConstants.AUTO_XY_PID.p(),
        SwerveConstants.AUTO_XY_PID.i(),
        SwerveConstants.AUTO_XY_PID.d()
    );
    private final PIDController rotPIDAuto = new PIDController(
        SwerveConstants.AUTO_ROT_PID.p(),
        SwerveConstants.AUTO_ROT_PID.i(),
        SwerveConstants.AUTO_ROT_PID.d()
    );

    private final ProfiledPIDController xPID = new ProfiledPIDController(
        SwerveConstants.XY_PID.p(),
        SwerveConstants.XY_PID.i(),
        SwerveConstants.XY_PID.d(),
        SwerveConstants.XY_CONSTRAINTS
    );
    private final ProfiledPIDController yPID = new ProfiledPIDController(
        SwerveConstants.XY_PID.p(),
        SwerveConstants.XY_PID.i(),
        SwerveConstants.XY_PID.d(),
        SwerveConstants.XY_CONSTRAINTS
    );
    private final ProfiledPIDController rotPID = new ProfiledPIDController(
        SwerveConstants.ROT_PID.p(),
        SwerveConstants.ROT_PID.i(),
        SwerveConstants.ROT_PID.d(),
        SwerveConstants.ROT_CONSTRAINTS
    );

    private final PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[] {
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new PhotonCamera("FrontLeft"),
            SwerveConstants.FRONT_LEFT_CAMERA
        ),
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new PhotonCamera("BackLeft"),
            SwerveConstants.BACK_LEFT_CAMERA
        ),
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new PhotonCamera("BackRight"),
            SwerveConstants.BACK_RIGHT_CAMERA
        ),
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new PhotonCamera("FrontRight"),
            SwerveConstants.FRONT_RIGHT_CAMERA
        ),
    };

    private final Pose2d[] photonLastPoses;

    private double NOTE_VELOCITY = Constants.NOTE_VELOCITY;

    private Pose3d speakerRotTarget = null;
    private boolean visionFallThrough = false;

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        super("Swerve Drive", SwerveConstants.CONFIG);
        rotPID.setIZone(SwerveConstants.ROT_PID.iZone());
        rotPID.enableContinuousInput(-Math.PI, Math.PI);

        photonLastPoses = new Pose2d[photonPoseEstimators.length];
        for (int i = 0; i < photonLastPoses.length; i++) {
            photonLastPoses[i] = new Pose2d();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("speakerX", () -> getSpeakerPosition().getX(), null);
        builder.addDoubleProperty("speakerY", () -> getSpeakerPosition().getY(), null);
        builder.addDoubleProperty("speakerDistance", this::getSpeakerDistance, null);
        builder.addBooleanProperty("inOpponentWing", this::inOpponentWing, null);
        builder.addDoubleProperty("tunableNoteVelocity", () -> NOTE_VELOCITY, velocity -> NOTE_VELOCITY = velocity);
        builder.addDoubleArrayProperty(
            "speakerRotTarget",
            () -> {
                Pose3d pose = speakerRotTarget != null
                    ? speakerRotTarget
                    : (Alliance.isBlue() ? Constants.BLUE_SPEAKER_3D : Constants.RED_SPEAKER_3D);
                return new double[] {
                    pose.getX(),
                    pose.getY(),
                    pose.getZ(),
                    pose.getRotation().getX(),
                    pose.getRotation().getY(),
                    pose.getRotation().getZ(),
                };
            },
            null
        );
    }

    @Override
    public void periodic() {
        updateOdometry(poseEstimator -> {
            Pose2d currentPose = getPosition();
            for (int i = 0; i < photonPoseEstimators.length; i++) {
                Optional<EstimatedRobotPose> pose = photonPoseEstimators[i].update();
                if (pose.isPresent()) {
                    Pose2d pose2d = pose.get().estimatedPose.toPose2d();
                    if (
                        visionFallThrough ||
                        currentPose.getTranslation().getDistance(pose2d.getTranslation()) < SwerveConstants.VISION_REJECT_DISTANCE
                    ) {
                        poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds);
                        field.getObject("PhotonVision/" + i).setPose(pose2d);
                        return;
                    }
                }

                field.getObject("PhotonVision/" + i).setPoses();
            }
        });
    }

    /**
     * Returns {@code true} if the robot is in the opponent's wing.
     */
    public boolean inOpponentWing() {
        return getPosition().getX() > Constants.OPPONENT_WING_LINE;
    }

    /**
     * Returns the distance from the speaker in meters, adjusted for the robot's movement.
     */
    public double getSpeakerDistance() {
        return getPosition().getTranslation().getDistance(getSpeakerPosition());
    }

    /**
     * Gets a field-relative position for the shot to the speaker
     * the robot should take, adjusted for the robot's movement.
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getSpeakerPosition() {
        Translation2d goalPose = Alliance.isBlue() ? Constants.BLUE_SPEAKER : Constants.RED_SPEAKER;
        ChassisSpeeds robotVel = getVelocity(true);
        double distanceToSpeaker = getPosition().getTranslation().getDistance(goalPose);
        double x = goalPose.getX() - (robotVel.vxMetersPerSecond * (distanceToSpeaker / NOTE_VELOCITY));
        double y = goalPose.getY() - (robotVel.vyMetersPerSecond * (distanceToSpeaker / NOTE_VELOCITY));
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
        // if (Math.random() > 0.9) System.out.println(shotRot.getDegrees()); lol

        Translation2d targetPosition = new Translation2d(
            speakerPosition.getX() + shotRot.getSin() * SwerveConstants.SPIN_COMPENSATION_X * speakerDistance,
            speakerPosition.getY() + shotRot.getCos() * SwerveConstants.SPIN_COMPENSATION_Y * speakerDistance
        );

        speakerRotTarget = new Pose3d(targetPosition.getX(), targetPosition.getY(), Constants.SPEAKER_HEIGHT, Math2.ROTATION3D_0);
        return MathUtil.angleModulus(targetPosition.minus(robotPoint).getAngle().getRadians() + Math.PI);
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
        return commandBuilder()
            .onInitialize(() -> rotPID.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> driveAngle(x.get(), y.get(), getSpeakerAngle(), rotPID, false));
    }

    /**
     * Drives to the amp.
     */
    public Command driveAmp() {
        return either(
            sequence(driveToPose(SwerveConstants.AMP_APPROACH_BLUE, true), driveToPose(SwerveConstants.AMP_SCORE_BLUE, true)),
            sequence(driveToPose(SwerveConstants.AMP_APPROACH_RED, true), driveToPose(SwerveConstants.AMP_SCORE_RED, true)),
            Alliance::isBlue
        );
    }

    /**
     * Allows the driver to keep driving, but forces the robot to face the stage.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     */
    public Command driveStage(Supplier<Double> x, Supplier<Double> y) {
        // TODO Fix
        return commandBuilder("swerve.alignWithStage")
            .onInitialize(() -> rotPID.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> {
                double stageAngle = Constants.STAGE.minus(getPosition().getTranslation()).getAngle().getRadians();
                double faceStageAngle;
                if (stageAngle <= Math.PI / 3 && stageAngle >= -Math.PI / 3) faceStageAngle = Math.PI; else if (
                    stageAngle >= Math.PI / 3 && stageAngle <= Math.PI
                ) faceStageAngle = -Math.PI / 3; else faceStageAngle = Math.PI / 3;

                driveAngle(x.get(), y.get(), faceStageAngle, rotPID, false);
            });
    }

    /**
     * Drives to a pose.
     * @param pose The pose to drive to.
     * @param willFinish If the command will finish after the robot has reached the pose.
     */
    private Command driveToPose(Pose2d pose, boolean willFinish) {
        return commandBuilder("swerve.alignWithAmp")
            .onInitialize(() -> {
                Pose2d robotPose = getPosition();
                ChassisSpeeds velocity = getVelocity(true);
                xPID.reset(robotPose.getX(), velocity.vxMetersPerSecond);
                yPID.reset(robotPose.getY(), velocity.vyMetersPerSecond);
                rotPID.reset(robotPose.getRotation().getRadians(), velocity.omegaRadiansPerSecond);
            })
            .onExecute(() -> {
                driveToPose(pose, xPID, yPID, rotPID, true);
            })
            .isFinished(() -> {
                Pose2d robotPose = getPosition();
                return (
                    willFinish &&
                    Math2.epsilonEquals(
                        0.0,
                        robotPose.getTranslation().getDistance(pose.getTranslation()),
                        SwerveConstants.POSE_XY_ERROR
                    ) &&
                    Math2.epsilonEquals(0.0, robotPose.getRotation().minus(pose.getRotation()).getRadians(), SwerveConstants.POSE_ROT_ERROR)
                );
            });
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
        return Choreo
            .choreoSwerveCommand(
                traj,
                this::getPosition,
                targetTime,
                this::getSpeakerAngle,
                rotPID,
                xPIDAuto,
                yPIDAuto,
                rotPIDAuto,
                speeds -> driveSpeeds(speeds, false, false),
                Alliance::isRed,
                this
            )
            .beforeStarting(() -> {
                if (resetOdometry) {
                    Pose2d initialPose = traj.getInitialPose();
                    zeroIMU(initialPose.getRotation());
                    resetOdometry(initialPose);
                }

                rotPID.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond);
                xPIDAuto.reset();
                yPIDAuto.reset();
                rotPIDAuto.reset();
            });
    }

    /**
     * While running, vision measurements outside of the allowed
     * deviation will be applied. Useful for practicing where the robot
     * is initialized at the wrong position, or if the robot has drifted
     * too far from its actual pose.
     */
    public Command visionFallThrough() {
        return new CommandBuilder("swerve.visionFallThrough()")
            .onInitialize(() -> visionFallThrough = true)
            .onEnd(() -> visionFallThrough = false)
            .ignoringDisable(true);
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
