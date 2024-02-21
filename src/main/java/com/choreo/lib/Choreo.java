// Copyright (c) Choreo contributors
// From Choreolib@https://github.com/SleipnirGroup/Choreo/commit/09b94f7b24969b4ca3d910439f0be1fb562834d5

package com.choreo.lib;

import com.google.gson.Gson;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Utilities to load and follow ChoreoTrajectories */
public class Choreo {

    private static final Gson gson = new Gson();

    /** Default constructor. */
    public Choreo() {}

    /**
     * Load a trajectory from the deploy directory. Choreolib expects .traj files to be placed in
     * src/main/deploy/choreo/[trajName].traj .
     *
     * @param trajName the path name in Choreo, which matches the file name in the deploy directory.
     *     Do not include ".traj" here.
     * @return the loaded trajectory, or null if the trajectory could not be loaded.
     */
    public static ChoreoTrajectory getTrajectory(String trajName) {
        var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        var traj_file = new File(traj_dir, trajName + ".traj");

        return loadFile(traj_file);
    }

    /**
     * Loads the split parts of the specified trajectory. Fails and returns null if any of the parts
     * could not be loaded.
     *
     * <p>This method determines the number of parts to load by counting the files that match the
     * pattern "trajName.X.traj", where X is a string of digits. Let this count be N. It then attempts
     * to load "trajName.1.traj" through "trajName.N.traj", consecutively counting up. If any of these
     * files cannot be loaded, the method returns null.
     *
     * @param trajName The path name in Choreo for this trajectory.
     * @return The ArrayList of segments, in order, or null.
     */
    public static ArrayList<ChoreoTrajectory> getTrajectoryGroup(String trajName) {
        // Count files matching the pattern for split parts.
        var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        File[] files = traj_dir.listFiles(file -> file.getName().matches(trajName + "\\.\\d+\\.traj"));
        int segmentCount = files.length;
        // Try to load the segments.
        var trajs = new ArrayList<ChoreoTrajectory>();
        for (int i = 1; i <= segmentCount; ++i) {
            File traj = new File(traj_dir, String.format("%s.%d.traj", trajName, i));
            ChoreoTrajectory trajectory = loadFile(traj);
            if (trajectory == null) {
                DriverStation.reportError("ChoreoLib: Missing segments for path group " + trajName, false);
                return null;
            }
            trajs.add(trajectory);
        }

        return trajs;
    }

    private static ChoreoTrajectory loadFile(File path) {
        try {
            var reader = new BufferedReader(new FileReader(path));
            ChoreoTrajectory traj = gson.fromJson(reader, ChoreoTrajectory.class);

            return traj;
        } catch (Exception ex) {
            DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
        }
        return null;
    }

    /**
     * Create a command to follow a Choreo path.
     *
     * @param trajectory The trajectory to follow. Use Choreo.getTrajectory(String trajName) to load
     *     this from the deploy directory.
     * @param poseSupplier A function that returns the current field-relative pose of the robot.
     * @param xController A PIDController for field-relative X translation (input: X error in meters,
     *     output: m/s).
     * @param yController A PIDController for field-relative Y translation (input: Y error in meters,
     *     output: m/s).
     * @param rotationController A PIDController for robot rotation (input: heading error in radians,
     *     output: rad/s). This controller will have its continuous input range set to -pi..pi by
     *     ChoreoLib.
     * @param outputChassisSpeeds A function that consumes the target robot-relative chassis speeds
     *     and commands them to the robot.
     * @param mirrorTrajectory If this returns true, the path will be mirrored to the opposite side,
     *     while keeping the same coordinate system origin. This will be called every loop during the
     *     command.
     * @param requirements The subsystem(s) to require, typically your drive subsystem only.
     * @return A command that follows a Choreo path.
     */
    public static Command choreoSwerveCommand(
        ChoreoTrajectory trajectory,
        Supplier<Pose2d> poseSupplier,
        double targetTime,
        Supplier<Double> targetAngle,
        ProfiledPIDController targetController,
        PIDController xController,
        PIDController yController,
        PIDController rotationController,
        Consumer<ChassisSpeeds> outputChassisSpeeds,
        BooleanSupplier mirrorTrajectory,
        Subsystem... requirements
    ) {
        ChoreoControlFunction controller = choreoSwerveController(xController, yController, rotationController);
        var timer = new Timer();
        return new FunctionalCommand(
            timer::restart,
            () -> {
                Pose2d pose = poseSupplier.get();
                ChassisSpeeds speeds = controller.apply(pose, trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean()));
                if (targetTime >= 0 && targetTime >= timer.get()) {
                    speeds =
                        new ChassisSpeeds(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            targetController.calculate(MathUtil.angleModulus(pose.getRotation().getRadians()), targetAngle.get())
                        );
                }

                outputChassisSpeeds.accept(speeds);
            },
            interrupted -> {
                timer.stop();
                if (interrupted) {
                    outputChassisSpeeds.accept(new ChassisSpeeds());
                } else {
                    outputChassisSpeeds.accept(trajectory.getFinalState().getChassisSpeeds());
                }
            },
            () -> timer.hasElapsed(trajectory.getTotalTime()),
            requirements
        );
    }

    /**
     * Creates a control function for following a ChoreoTrajectoryState.
     *
     * @param xController A PIDController for field-relative X translation (input: X error in meters,
     *     output: m/s).
     * @param yController A PIDController for field-relative Y translation (input: Y error in meters,
     *     output: m/s).
     * @param rotationController A PIDController for robot rotation (input: heading error in radians,
     *     output: rad/s). This controller will have its continuous input range set to -pi..pi by
     *     ChoreoLib.
     * @return A ChoreoControlFunction to track ChoreoTrajectoryStates. This function returns
     *     robot-relative ChassisSpeeds.
     */
    public static ChoreoControlFunction choreoSwerveController(
        PIDController xController,
        PIDController yController,
        PIDController rotationController
    ) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return (pose, referenceState) -> {
            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double rotationFF = referenceState.angularVelocity;

            double xFeedback = xController.calculate(pose.getX(), referenceState.x);
            double yFeedback = yController.calculate(pose.getY(), referenceState.y);
            double rotationFeedback = rotationController.calculate(pose.getRotation().getRadians(), referenceState.heading);

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                rotationFF + rotationFeedback,
                pose.getRotation()
            );
        };
    }
}
