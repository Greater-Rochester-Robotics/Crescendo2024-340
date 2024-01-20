package org.team340.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.config.rev.SparkPIDControllerConfig;
import org.team340.robot.Constants.RobotMap;
import org.team340.robot.Constants.ShooterConstants;
import org.team340.robot.Constants.ShooterConstants.PivotConstants;

public class Shooter extends GRRSubsystem {

    private final CANSparkMax pivotMotor = createSparkMax("Pivot Motor", RobotMap.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax feedMotor = createSparkMax("Feeder Motor", RobotMap.SHOOTER_FEED_MOTOR, MotorType.kBrushless);
    private final CANSparkFlex leftShootMotor = createSparkFlex(
        "Shooter Motor Left",
        RobotMap.SHOOTER_LEFT_SHOOT_MOTOR,
        MotorType.kBrushless
    );
    private final CANSparkFlex rightShootMotor = createSparkFlex(
        "Shooter Motor Right",
        RobotMap.SHOOTER_RIGHT_SHOOT_MOTOR,
        MotorType.kBrushless
    );

    private final SparkPIDController pivotPID = pivotMotor.getPIDController();
    private final SparkPIDController feedPID = feedMotor.getPIDController();
    private final SparkPIDController leftShootPID = leftShootMotor.getPIDController();
    private final SparkPIDController rightShootPID = rightShootMotor.getPIDController();

    private final AbsoluteEncoder pivotAbsoluteEncoder = createSparkMaxAbsoluteEncoder(
        "Pivot Absolute Encoder",
        pivotMotor,
        Type.kDutyCycle
    );
    private final RelativeEncoder pivotRelativeEncoder = pivotMotor.getEncoder();

    private final DigitalInput noteDetector = createDigitalInput("Note Detector", RobotMap.SHOOTER_NOTE_DETECTOR);

    public Shooter() {
        super("Shooter");
        pivotRelativeEncoder.setPositionConversionFactor(PivotConstants.PIVOT_REL_ENC_CONVERSION);
        pivotAbsoluteEncoder.setPositionConversionFactor(Math2.TWO_PI);

        new SparkPIDControllerConfig()
            .setPID(PivotConstants.PIVOT_PID.p(), PivotConstants.PIVOT_PID.i(), PivotConstants.PIVOT_PID.d())
            .setIZone(PivotConstants.PIVOT_PID.iZone())
            .setOutputRange(PivotConstants.PIVOT_PID_MIN_OUTPUT, PivotConstants.PIVOT_PID_MAX_OUTPUT)
            .apply(pivotMotor, pivotPID);
        pivotPID.setSmartMotionMaxVelocity(PivotConstants.PIVOT_MAX_VEL, 0);
        pivotPID.setSmartMotionMaxVelocity(PivotConstants.PIVOT_MAX_VEL, 0);
        pivotPID.setSmartMotionMinOutputVelocity(PivotConstants.PIVOT_MAX_VEL, 0);
        pivotPID.setSmartMotionMaxAccel(PivotConstants.PIVOT_MAX_ACCEL, 0);
        pivotPID.setSmartMotionAllowedClosedLoopError(PivotConstants.PIVOT_CLOSED_LOOP_ERR, 0);

        new SparkPIDControllerConfig()
            .setPID(ShooterConstants.SHOOTER_FEED_PID.p(), ShooterConstants.SHOOTER_FEED_PID.i(), ShooterConstants.SHOOTER_FEED_PID.d())
            .setIZone(ShooterConstants.SHOOTER_FEED_PID.iZone())
            .apply(feedMotor, feedPID);

        new SparkPIDControllerConfig()
            .setPID(
                ShooterConstants.SHOOTER_LEFT_SHOOT_PID.p(),
                ShooterConstants.SHOOTER_LEFT_SHOOT_PID.i(),
                ShooterConstants.SHOOTER_LEFT_SHOOT_PID.d()
            )
            .setIZone(ShooterConstants.SHOOTER_LEFT_SHOOT_PID.iZone())
            .apply(leftShootMotor, leftShootPID);

        new SparkPIDControllerConfig()
            .setPID(
                ShooterConstants.SHOOTER_RIGHT_SHOOT_PID.p(),
                ShooterConstants.SHOOTER_RIGHT_SHOOT_PID.i(),
                ShooterConstants.SHOOTER_RIGHT_SHOOT_PID.d()
            )
            .setIZone(ShooterConstants.SHOOTER_RIGHT_SHOOT_PID.iZone())
            .apply(rightShootMotor, rightShootPID);
    }

    /**
     * This starts running the pivot motor to an angle. This must run regularly until the shooter reaches the angle.
     * @param angleToShootAt This is the angle that will be used.
     */
    private void setShooterToAngle(double angleToShootAt) {
        if (angleToShootAt < PivotConstants.PIVOT_MINIMUM_ANGLE || angleToShootAt > PivotConstants.PIVOT_MAXIMUM_ANGLE) {
            DriverStation.reportWarning(
                "Invalid shooter pivot angle. " +
                angleToShootAt +
                " is not between " +
                PivotConstants.PIVOT_MINIMUM_ANGLE +
                " and " +
                PivotConstants.PIVOT_MAXIMUM_ANGLE,
                false
            );
            return;
        }
        pivotPID.setReference(distanceToMoveDart(angleToShootAt), ControlType.kSmartMotion);
    }

    /**
     * Internal function which converts the current angle (in radians) from absolute encoder to the extension length (in inches) the dart needs to move to
     * @param sensorAngle Angle from the pivot motor's absolute encoder
     * @return The extension length the dart needs to move to
     */
    private double dartExtensionFromAngle(double sensorAngle) {
        double totalLengthOfDart = Math.sqrt(
            PivotConstants.SUM_OF_SQUARES_OF_LENGTHS -
            PivotConstants.PIVOT_TWICE_THE_PRODUCT_OF_LENGTHS *
            Math.cos(sensorAngle + PivotConstants.PIVOT_OFFSET_ANGLE)
        );
        return (totalLengthOfDart - PivotConstants.PIVOT_MINIMUM_LENGTH_OF_DART);
    }

    /**
     * Internal function which returns distance (in inches) the dart needs to be moved based on current angle (from relative encoder) and a target angle
     * @param targetAngle Pivot angle the dart needs to be at
     * @return distance the dart needs to be moved
     */
    private double distanceToMoveDart(double targetAngle) {
        double deltaExtensionLength = dartExtensionFromAngle(targetAngle) - dartExtensionFromAngle(pivotAbsoluteEncoder.getPosition());
        return deltaExtensionLength + pivotRelativeEncoder.getPosition();
    }

    /**
     * This starts running the shooter motors to there respective speeds..
     * @param leftSpeed This is speed the left motor will be set to.
     * @param rightSpeed This is speed the right motor will be set to.
     */
    private void setShooterToSpeed(double leftSpeed, double rightSpeed) {
        leftShootPID.setReference(leftSpeed, ControlType.kVelocity);
        rightShootPID.setReference(rightSpeed, ControlType.kVelocity);
    }

    /**
     * this method shoots the note by spinning the feedMotor. Make sure to stop the feed motor after the note has completely cleared the shooter.
     * @param feederWheelSpeed This is the speed the shooter wheels will attempt to reach. Make this big so you do not rip the note.
     */
    private void shoot(double feederWheelSpeed) {
        feedPID.setReference(feederWheelSpeed, ControlType.kVelocity);
    }

    private boolean hasShooterReachedAngle() {
        return true;
    }

    private boolean hasShooterReachedSpeed() {
        return true;
    }

    /**
     * This command receives a note from the intake.
     * @return A Command for receiving a note.
     */
    public Command receiveNote() {
        return commandBuilder()
            .onInitialize(() -> {
                feedPID.setReference(ShooterConstants.FEED_INTAKE_SPEED, ControlType.kDutyCycle);
            })
            .isFinished(() -> {
                return noteDetector.get();
            })
            .onEnd(() -> {
                feedMotor.stopMotor();
            });
    }

    public Command shooterToAngle(double angleToShootAt) {
        return shooterToAngle(() -> angleToShootAt);
    }

    /**
     * drives the shooter to the specified angle. Since the angle is a supplier it may change while this command is executing.
     * @param angleToShootAt This is the angle the shooter will go to, check {@link #hasShooterReachedAngle()} to see if it has finished.
     * @return
     */
    public Command shooterToAngle(Supplier<Double> angleToShootAt) {
        return commandBuilder().onExecute(() -> setShooterToAngle(angleToShootAt.get())).onEnd(() -> pivotMotor.stopMotor());
    }

    /**
     * Runs the shooter up to a specified speed. This speed can change while this command is running.
     * @param shooterSpeed This is the speed to drive the shooter at.
     * @return Returns this command.
     */
    public Command setShootSpeed(Supplier<Double> shooterSpeed) {
        return commandBuilder()
            .onExecute(() -> {
                setShooterToSpeed(shooterSpeed.get(), shooterSpeed.get());
            })
            .onEnd(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }

    /**
     * This command takes the current position, uses it to decide the angle to shoot at, then shoots the note.
     * Note that this will not correct the robot's yaw or position.
     * @param robotPosition This is the position used for the math, note that it must be a supplier.
     * @return This command.
     */
    public Command shootSpeaker(Supplier<Pose2d> robotPosition) {
        Mutable<Integer> counter = new Mutable<>(0);

        return commandBuilder()
            .onInitialize(() -> {
                //TODO: find a value that works well
                counter.set(40);
                //TODO: do math!!!
                setShooterToSpeed(0, 0);
                setShooterToAngle(0);
            })
            .onExecute(() -> {
                if (hasShooterReachedAngle() && hasShooterReachedSpeed()) {
                    //TODO: make a value for this
                    shoot(0);
                    counter.set(Math.max(0, counter.get() - 1));
                }
            })
            .isFinished(() -> counter.get() == 0)
            .onEnd(() -> {
                pivotMotor.stopMotor();
                feedMotor.stopMotor();
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }

    public Command shootSpeaker(Pose2d robotPosition) {
        return shootSpeaker(() -> robotPosition);
    }

    public Command shootAmp() {
        //TODO: write this
        return null;
    }

    public Command shootTrap() {
        //TODO: write this
        return null;
    }

    public Command spit() {
        //TODO: write this
        return null;
    }
}
