package org.team340.robot.subsystems;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

/**
 * The swerve subsystem.
 */
public class Swerve extends SwerveBase {

    private final PIDController autoXPID = new PIDController(
        SwerveConstants.AUTO_XY_PID.p(),
        SwerveConstants.AUTO_XY_PID.i(),
        SwerveConstants.AUTO_XY_PID.d()
    );
    private final PIDController autoYPID = new PIDController(
        SwerveConstants.AUTO_XY_PID.p(),
        SwerveConstants.AUTO_XY_PID.i(),
        SwerveConstants.AUTO_XY_PID.d()
    );
    private final PIDController autoRotPID = new PIDController(
        SwerveConstants.AUTO_ROT_PID.p(),
        SwerveConstants.AUTO_ROT_PID.i(),
        SwerveConstants.AUTO_ROT_PID.d()
    );

    private final ProfiledPIDController rotController = new ProfiledPIDController(
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
            new Transform3d(
                new Translation3d(-0.29356304, 0.27327352, 0.23771098),
                new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(-170.0))
            )
        ),
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new PhotonCamera("BackRight"),
            new Transform3d(
                new Translation3d(-0.29356304, -0.27327352, 0.23771098),
                new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(170.0))
            )
        ),
    };

    private double NOTE_VELOCITY = 20.0;

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        super("Swerve Drive", SwerveConstants.CONFIG);
        rotController.setIZone(SwerveConstants.ROT_PID.iZone());
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        resetOdometry(new Pose2d(0.5, 7.9, Math2.ROTATION2D_0));
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("note velocity", () -> NOTE_VELOCITY, velocity -> NOTE_VELOCITY = velocity);
        builder.addDoubleProperty("Shot X", () -> getShotPosition().getX(), null);
        builder.addDoubleProperty("Shot Y", () -> getShotPosition().getY(), null);
        builder.addDoubleProperty("speakerDistance", this::getDistanceToSpeaker, null);
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
     * This command gets the distance to the speaker of the current alliance.
     * @return The distance.
     */
    public double getDistanceToSpeaker() {
        return getPosition().getTranslation().getDistance(getShotPosition());
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
     * Drives the robot using percents of its calculated max velocity while locked at a field relative angle.
     * @param x X speed.
     * @param y Y speed.
     * @param angle The desired field relative angle to point at in radians.
     */
    public Command driveAngle(Supplier<Double> x, Supplier<Double> y, double angle) {
        return commandBuilder("swerve.driveAngle(" + Math2.toFixed(angle) + ")")
            .onInitialize(() -> rotController.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> driveAngle(x.get(), y.get(), angle, rotController));
    }

    /**
     * This command allows the driver to keep driving, but forces the robot to face the speaker.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     */
    public Command driveOnTarget(Supplier<Double> x, Supplier<Double> y) {
        return commandBuilder()
            .onInitialize(() -> rotController.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> driveAroundPoint(x.get(), y.get(), Math.PI, getShotPosition(), rotController));
    }

    public Translation2d getShotPosition() {
        boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
        Translation2d goalPose = isBlueAlliance ? Constants.BLUE_SPEAKER : Constants.RED_SPEAKER;
        ChassisSpeeds robotVel = getVelocity(true);
        double distanceToSpeaker = getPosition().getTranslation().getDistance(goalPose);
        double x = goalPose.getX() - (robotVel.vxMetersPerSecond * (distanceToSpeaker / NOTE_VELOCITY));
        double y = goalPose.getY() - (robotVel.vyMetersPerSecond * (distanceToSpeaker / NOTE_VELOCITY));
        return new Translation2d(x, y);
    }

    /**
     * This command aligns the robot with the respective side of their stage.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     */
    public Command alignWithStage(Supplier<Double> x, Supplier<Double> y) {
        return commandBuilder("swerve.alignWithStage")
            .onInitialize(() -> rotController.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> {
                double angleBetweenRobotAndStage = Constants.STAGE.minus(getPosition().getTranslation()).getAngle().getRadians();
                double angleToFaceStage;
                if (angleBetweenRobotAndStage <= Math.PI / 3 && angleBetweenRobotAndStage >= -Math.PI / 3) angleToFaceStage =
                    Math.PI; else if (angleBetweenRobotAndStage >= Math.PI / 3 && angleBetweenRobotAndStage <= Math.PI) angleToFaceStage =
                    -Math.PI / 3; else angleToFaceStage = Math.PI / 3;

                driveAngle(x.get(), y.get(), angleToFaceStage, rotController);
            });
    }

    public Command alignWithAmp(Supplier<Double> x, Supplier<Double> y) {
        return commandBuilder("swerve.alignWithAmp")
            .onInitialize(() -> rotController.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> {
                double angle = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? Math2.HALF_PI : -Math2.HALF_PI;
                driveAngle(x.get(), y.get(), angle, rotController);
            });
    }

    public Command followTrajectory(ChoreoTrajectory traj) {
        return followTrajectory(traj, false);
    }

    public Command followTrajectory(ChoreoTrajectory traj, boolean isFirst) {
        return Choreo
            .choreoSwerveCommand(
                traj,
                this::getPosition,
                autoXPID,
                autoYPID,
                autoRotPID,
                speeds -> driveSpeeds(speeds, false, false),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red),
                this
            )
            .beforeStarting(() -> {
                if (isFirst) {
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
}
