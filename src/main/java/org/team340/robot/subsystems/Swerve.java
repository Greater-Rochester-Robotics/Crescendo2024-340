package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team340.lib.swerve.SwerveBase;
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
            new PhotonCamera("BackLeft"),
            SwerveConstants.BACK_LEFT_CAMERA
        ),
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new PhotonCamera("BackRight"),
            SwerveConstants.BACK_RIGHT_CAMERA
        ),
    };

    private double NOTE_VELOCITY = Constants.NOTE_VELOCITY;

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        super("Swerve Drive", SwerveConstants.CONFIG);
        rotPID.setIZone(SwerveConstants.ROT_PID.iZone());
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        resetOdometry(new Pose2d(0.5, 7.9, Math2.ROTATION2D_0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("speakerX", () -> getSpeakerPosition().getX(), null);
        builder.addDoubleProperty("speakerY", () -> getSpeakerPosition().getY(), null);
        builder.addDoubleProperty("speakerDistance", this::getSpeakerDistance, null);
        builder.addDoubleProperty("tunableNoteVelocity", () -> NOTE_VELOCITY, velocity -> NOTE_VELOCITY = velocity);
    }

    @Override
    public void periodic() {
        updateOdometry(poseEstimator -> {
            for (PhotonPoseEstimator photonPoseEstimator : photonPoseEstimators) {
                Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
                if (pose.isPresent()) poseEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
            }
        });
    }

    /**
     * This command gets the distance of the current shot to the speaker.
     * @return The distance in meters.
     */
    public double getSpeakerDistance() {
        return getPosition().getTranslation().getDistance(getSpeakerPosition());
    }

    /**
     * Gets a field-relative position for the shot to the speaker the robot should take.
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getSpeakerPosition() {
        boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
        Translation2d goalPose = isBlueAlliance ? Constants.BLUE_SPEAKER : Constants.RED_SPEAKER;
        ChassisSpeeds robotVel = getVelocity(true);
        double distanceToSpeaker = getPosition().getTranslation().getDistance(goalPose);
        double x = goalPose.getX() - (robotVel.vxMetersPerSecond * (distanceToSpeaker / NOTE_VELOCITY));
        double y = goalPose.getY() - (robotVel.vyMetersPerSecond * (distanceToSpeaker / NOTE_VELOCITY));
        return new Translation2d(x, y);
    }

    /**
     * Zeroes the IMU to a specified yaw.
     */
    public Command zeroIMU(Rotation2d yaw) {
        return runOnce(() -> imu.setZero(yaw)).withName("swerve.zero(" + yaw.toString() + ")");
    }

    /**
     * Drives the robot as a percent of its max velocity (inputs are from {@code -1.0} to {@code 1.0}).
     * @param x X speed.
     * @param y Y speed.
     * @param rot Rotational speed.
     * @param fieldRelative If the robot should drive field relative.
     */
    public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot, boolean fieldRelative) {
        return commandBuilder("swerve.drive()").onExecute(() -> drive(x.get(), y.get(), rot.get(), fieldRelative));
    }

    /**
     * Allows the driver to keep driving, but forces the robot to face the speaker.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     */
    public Command driveSpeaker(Supplier<Double> x, Supplier<Double> y) {
        return commandBuilder()
            .onInitialize(() -> rotPID.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> driveAroundPoint(x.get(), y.get(), Math.PI, getSpeakerPosition(), rotPID, false));
    }

    /**
     * Drives to the amp.
     */
    public Command driveAmp() {
        return either(
            sequence(driveToPose(SwerveConstants.AMP_APPROACH_BLUE, true), driveToPose(SwerveConstants.AMP_SCORE_BLUE, false)),
            sequence(driveToPose(SwerveConstants.AMP_APPROACH_RED, true), driveToPose(SwerveConstants.AMP_SCORE_RED, false)),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)
        );
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
                double stageAngle = Constants.STAGE.minus(getPosition().getTranslation()).getAngle().getRadians();
                double faceStageAngle;
                if (stageAngle <= Math.PI / 3 && stageAngle >= -Math.PI / 3) faceStageAngle = Math.PI; else if (
                    stageAngle >= Math.PI / 3 && stageAngle <= Math.PI
                ) faceStageAngle = -Math.PI / 3; else faceStageAngle = Math.PI / 3;

                driveAngle(x.get(), y.get(), faceStageAngle, rotPID, false);
            });
    }

    /**
     * Follows a trajectory.
     * @param traj The trajectory to follow.
     */
    public Command followTrajectory(ChoreoTrajectory traj) {
        return followTrajectory(traj, false);
    }

    /**
     * Follows a trajectory.
     * @param traj The trajectory to follow.
     * @param resetOdometry If the odometry should be reset to the first pose in the trajectory.
     */
    public Command followTrajectory(ChoreoTrajectory traj, boolean resetOdometry) {
        return Choreo
            .choreoSwerveCommand(
                traj,
                this::getPosition,
                xPIDAuto,
                yPIDAuto,
                rotPIDAuto,
                speeds -> driveSpeeds(speeds, false, false),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red),
                this
            )
            .beforeStarting(() -> {
                if (resetOdometry) {
                    Pose2d initialPose = traj.getInitialPose();
                    resetOdometry(initialPose);
                    zeroIMU(initialPose.getRotation());
                }
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
}
