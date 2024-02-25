package org.team340.lib.swerve.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.team340.lib.swerve.config.SwerveConfig;

/**
 * An extension of WPILib's {@link Field2d} with a helper method easily update the visualization.
 */
public class SwerveField2d extends Field2d {

    private final SwerveConfig config;

    /**
     * Create the field.
     * @param config The general swerve config.
     * @param moduleTranslations Translations representing the locations of the swerve modules.
     */
    public SwerveField2d(SwerveConfig config) {
        this.config = config;
    }

    /**
     * Update the field.
     * @param newPose The robot's new pose.
     */
    public void update(Pose2d newPose) {
        setRobotPose(newPose);
    }
}
