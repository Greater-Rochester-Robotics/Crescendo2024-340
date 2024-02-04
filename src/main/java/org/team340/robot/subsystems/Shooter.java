package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.RobotMap;
import org.team340.robot.Constants.ShooterConstants;

// TODO Docs
public class Shooter extends GRRSubsystem {

    private final CANSparkFlex leftShootMotor;
    private final CANSparkFlex rightShootMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final SparkPIDController leftShootPID;
    private final SparkPIDController rightShootPID;

    private double leftTargetSpeed = 0.0;
    private double rightTargetSpeed = 0.0;

    public Shooter() {
        super("Shooter");
        leftShootMotor = createSparkFlex("Left Motor", RobotMap.SHOOTER_SHOOT_LEFT_MOTOR, MotorType.kBrushless);
        rightShootMotor = createSparkFlex("Right Motor", RobotMap.SHOOTER_SHOOT_RIGHT_MOTOR, MotorType.kBrushless);
        leftEncoder = leftShootMotor.getEncoder();
        rightEncoder = rightShootMotor.getEncoder();
        leftShootPID = leftShootMotor.getPIDController();
        rightShootPID = rightShootMotor.getPIDController();

        ShooterConstants.SHOOT_LEFT_MOTOR_CONFIG.apply(leftShootMotor);
        ShooterConstants.SHOOT_RIGHT_MOTOR_CONFIG.apply(rightShootMotor);
        ShooterConstants.SHOOTER_ENC_CONFIG.apply(leftShootMotor, leftEncoder);
        ShooterConstants.SHOOTER_ENC_CONFIG.apply(rightShootMotor, rightEncoder);
        ShooterConstants.SHOOT_PID_CONFIG.apply(leftShootMotor, leftShootPID);
        ShooterConstants.SHOOT_PID_CONFIG.apply(rightShootMotor, rightShootPID);
    }

    // TODO Should this be private?
    // TODO Revisit naming
    /**
     * This starts running the shooter motors to there respective speeds..
     * @param leftSpeed This is speed the left motor will be set to.
     * @param rightSpeed This is speed the right motor will be set to.
     */
    public void setShooterToSpeed(double speed) {
        double leftSpeed = speed * ShooterConstants.LEFT_TO_RIGHT_RATIO;
        double rightSpeed = speed;
        leftTargetSpeed = leftSpeed;
        rightTargetSpeed = rightSpeed;
        leftShootPID.setReference(leftSpeed, ControlType.kVelocity);
        rightShootPID.setReference(rightSpeed, ControlType.kVelocity);
    }

    // TODO Shorten name?
    /**
     * This method checks if the speed of the shooter motors is within a tolerance of the setpoint.
     * @return whether the shooter motors have reached their setpoints.
     */
    public boolean hasShooterReachedSpeed() {
        return (
            Math2.epsilonEquals(leftTargetSpeed, leftEncoder.getVelocity(), ShooterConstants.SPEED_TOLERANCE) &&
            Math2.epsilonEquals(rightTargetSpeed, rightEncoder.getVelocity(), ShooterConstants.SPEED_TOLERANCE)
        );
    }

    // TODO Docs
    public Command setShootSpeed(double shooterSpeed) {
        return setShootSpeed(() -> shooterSpeed);
    }

    // TODO This should stop the motors when it ends
    /**
     * Sets the speed of the shooter.
     * @param shooterSpeed This is the speed to drive the shooter at.
     */
    public Command setShootSpeed(Supplier<Double> shooterSpeed) {
        return commandBuilder("shooter.setShootSpeed()").onExecute(() -> setShooterToSpeed(shooterSpeed.get()));
    }

    // TODO Shouldn't be needed anymore, setShootSpeed(0.0) can be used
    /**
     * A command that stops all motors, and keeps running until it is interrupted.
     * This should be set as the default command.
     * @return This command.
     */
    public Command stopShooter() {
        return commandBuilder("shooter.stopShooter()")
            .onInitialize(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }

    /**
     * Spits the note out of the shooter in case it is stuck.
     */
    public Command spit() {
        return commandBuilder("shooter.spit()")
            .onInitialize(() -> {
                leftShootPID.setReference(ShooterConstants.LEFT_SPIT_SPEED, ControlType.kDutyCycle);
                rightShootPID.setReference(ShooterConstants.RIGHT_SPIT_SPEED, ControlType.kDutyCycle);
            })
            .onEnd(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }
}
