package org.team340.robot;

import edu.wpi.first.math.geometry.Translation2d;
import org.team340.lib.controller.ControllerConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double kPeriod = 0.020;
    public static final double kVoltage = 12.0;

    public static final ControllerConfig kDriver = new ControllerConfig()
        .setPort(0)
        .setDeadbands(0.15, 0.05)
        .setThresholds(0.5, 0.05);

    public static final ControllerConfig kCoDriver = new ControllerConfig()
        .setPort(1)
        .setDeadbands(0.15, 0.05)
        .setThresholds(0.5, 0.05);

    public static final class RobotMap {

        public static final int kMoveFL = 2;
        public static final int kTurnFL = 3;
        public static final int kMoveBL = 4;
        public static final int kTurnBL = 5;
        public static final int kMoveBR = 6;
        public static final int kTurnBR = 7;
        public static final int kMoveFR = 8;
        public static final int kTurnFR = 9;

        public static final int kIntakePivotMotor = 20;
        public static final int kIntakeRollerMotor = 21;

        public static final int kPivotMotor = 30;
        public static final int kFeederMotor = 31;
        public static final int kShooterLeftMotor = 32;
        public static final int kShooterRightMotor = 33;

        public static final int kPivotLimit = 8;
        public static final int kNoteDetector = 9;
    }

    public static final class FieldConstants {

        public static final double kLength = 16.541;
        public static final double kWidth = 8.211;

        public static final Translation2d kBlueSpeaker = new Translation2d(-0.04, 5.5479);
        public static final Translation2d kRedSpeaker = new Translation2d(
            kLength - kBlueSpeaker.getX(),
            kBlueSpeaker.getY()
        );

        public static final Translation2d kBlueFeed = new Translation2d(0.0, 7.4);
        public static final Translation2d kRedFeed = new Translation2d(kLength - kBlueFeed.getX(), kBlueFeed.getY());

        public static final double kFenderShotDistance = 1.4;
    }
}
