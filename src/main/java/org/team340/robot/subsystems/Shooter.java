package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;
import org.team340.robot.Constants.ShooterConstants;

public class Shooter extends GRRSubsystem {

    private final CANSparkMax feedMotor = createSparkMax("Feeder Motor", RobotMap.SHOOTER_FEED_MOTOR, MotorType.kBrushless);
    private final CANSparkFlex leftShootMotor = createSparkFlex(
        "Shooter Motor Left",
        RobotMap.SHOOTER_SHOOT_LEFT_MOTOR,
        MotorType.kBrushless
    );
    private final CANSparkFlex rightShootMotor = createSparkFlex(
        "Shooter Motor Right",
        RobotMap.SHOOTER_SHOOT_RIGHT_MOTOR,
        MotorType.kBrushless
    );

    private final SparkPIDController feedPID = feedMotor.getPIDController();
    private final SparkPIDController leftShootPID = leftShootMotor.getPIDController();
    private final SparkPIDController rightShootPID = rightShootMotor.getPIDController();

    private final DigitalInput noteDetector = createDigitalInput("Note Detector", RobotMap.SHOOTER_NOTE_DETECTOR);

    public Shooter() {
        super("Shooter");
        ShooterConstants.FEED_MOTOR_CONFIG.apply(feedMotor);
        ShooterConstants.FEED_PID_CONFIG.apply(feedMotor, feedPID);

        ShooterConstants.SHOOT_LEFT_MOTOR_CONFIG.apply(leftShootMotor);
        ShooterConstants.SHOOT_PID_CONFIG.apply(leftShootMotor, leftShootPID);

        ShooterConstants.SHOOT_RIGHT_MOTOR_CONFIG.apply(rightShootMotor);
        ShooterConstants.SHOOT_PID_CONFIG.apply(rightShootMotor, rightShootPID);
    }

    /**
     * This starts running the shooter motors to there respective speeds..
     * @param leftSpeed This is speed the left motor will be set to.
     * @param rightSpeed This is speed the right motor will be set to.
     */
    public void setShooterToSpeed(double leftSpeed, double rightSpeed) {
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

    public boolean hasShooterReachedSpeed() {
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

    /**
     * Runs the shooter up to a specified speed. This speed can change while this command is running.
     * @param shooterSpeed This is the speed to drive the shooter at.
     * @return Returns this command.
     */
    public Command setShootSpeed(Supplier<Double> shooterSpeed) {
        return setShootSpeed(shooterSpeed, true);
    }

    /**
     * Runs the shooter up to a specified speed. This speed can change while this command is running.
     * @param shooterSpeed This is the speed to drive the shooter at.
     * @param willFinish If this is false, the command will never finish on it's own.
     * @return Returns this command.
     */
    public Command setShootSpeed(Supplier<Double> shooterSpeed, boolean willFinish) {
        return commandBuilder()
            .onExecute(() -> {
                setShooterToSpeed(shooterSpeed.get(), shooterSpeed.get());
            })
            .onEnd(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }

    public Command spit() {
        //TODO: write this
        return null;
    }
}
