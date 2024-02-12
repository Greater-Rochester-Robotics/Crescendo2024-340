package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.RobotMap;
import org.team340.robot.Constants.ShooterConstants;

/**
 * The shooter subsystem.
 */
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

        ShooterConstants.Configs.LEFT_MOTOR.apply(leftShootMotor);
        ShooterConstants.Configs.RIGHT_MOTOR.apply(rightShootMotor);
        ShooterConstants.Configs.ENCODER.apply(leftShootMotor, leftEncoder);
        ShooterConstants.Configs.ENCODER.apply(rightShootMotor, rightEncoder);
        ShooterConstants.Configs.PID.apply(leftShootMotor, leftShootPID);
        ShooterConstants.Configs.PID.apply(rightShootMotor, rightShootPID);
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("left-target", () -> leftTargetSpeed, null);
        builder.addDoubleProperty("right-target", () -> rightTargetSpeed, null);
    }

    /**
     * This starts running the shooter motors to their respective speeds.
     * @param speed This is speed the right motor will be set to with the left motor set to {@link ShooterConstants#RIGHT_TO_LEFT_RATIO} times this.
     */
    private void applySpeed(double speed) {
        double leftSpeed = speed;
        double rightSpeed = speed * ShooterConstants.RIGHT_TO_LEFT_RATIO;
        leftTargetSpeed = leftSpeed;
        rightTargetSpeed = rightSpeed;
        leftShootPID.setReference(leftSpeed, ControlType.kVelocity);
        rightShootPID.setReference(rightSpeed, ControlType.kVelocity);
    }

    /**
     * This method checks if the speed of the shooter motors is within a tolerance of the setpoint.
     * @return whether the shooter motors have reached their setpoints.
     */
    public boolean hasReachedSpeed() {
        return (
            Math2.epsilonEquals(leftTargetSpeed, leftEncoder.getVelocity(), ShooterConstants.SPEED_TOLERANCE) &&
            Math2.epsilonEquals(rightTargetSpeed, rightEncoder.getVelocity(), ShooterConstants.SPEED_TOLERANCE)
        );
    }

    /**
     * Sets the speed of the shooter.
     * @param shooterSpeed This is the speed to drive the shooter at.
     */
    public Command setSpeed(double shooterSpeed) {
        return setSpeed(() -> shooterSpeed);
    }

    /**
     * Sets the speed of the shooter.
     * @param shooterSpeed This is the speed to drive the shooter at.
     */
    public Command setSpeed(Supplier<Double> shooterSpeed) {
        return commandBuilder("shooter.setShootSpeed()")
            .onExecute(() -> applySpeed(shooterSpeed.get()))
            .onEnd(() -> {
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
                leftShootMotor.set(ShooterConstants.LEFT_SPIT_SPEED);
                rightShootMotor.set(ShooterConstants.RIGHT_SPIT_SPEED);
            })
            .onEnd(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }
}
