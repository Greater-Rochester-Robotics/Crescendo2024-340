package org.team340.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    private final SimpleMotorFeedforward feedforward;

    private final SysIdRoutine sysIdRoutine;
    private final MutableMeasure<Voltage> sysIdAppliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> sysIdPosition = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> sysIdVelocity = mutable(RotationsPerSecond.of(0));

    private boolean pidActive = false;
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
        feedforward =
            new SimpleMotorFeedforward(
                ShooterConstants.Configs.FEED_FORWARD.s(),
                ShooterConstants.Configs.FEED_FORWARD.v(),
                ShooterConstants.Configs.FEED_FORWARD.a()
            );

        sysIdRoutine =
            new SysIdRoutine(
                ShooterConstants.Configs.SYSID,
                new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> {
                        leftShootMotor.setVoltage(volts.in(Volts));
                        rightShootMotor.setVoltage(volts.in(Volts));
                    },
                    log -> {
                        log
                            .motor("shooter-left")
                            .voltage(
                                sysIdAppliedVoltage.mut_replace(
                                    leftShootMotor.getAppliedOutput() * RobotController.getBatteryVoltage(),
                                    Volts
                                )
                            )
                            .angularPosition(sysIdPosition.mut_replace(leftEncoder.getPosition(), Rotations))
                            .angularVelocity(sysIdVelocity.mut_replace(leftEncoder.getVelocity() / 60.0, RotationsPerSecond));

                        log
                            .motor("shooter-right")
                            .voltage(
                                sysIdAppliedVoltage.mut_replace(
                                    rightShootMotor.getAppliedOutput() * RobotController.getBatteryVoltage(),
                                    Volts
                                )
                            )
                            .angularPosition(sysIdPosition.mut_replace(rightEncoder.getPosition(), Rotations))
                            .angularVelocity(sysIdVelocity.mut_replace(rightEncoder.getVelocity() / 60.0, RotationsPerSecond));
                    },
                    this
                )
            );

        ShooterConstants.Configs.LEFT_MOTOR.apply(leftShootMotor);
        ShooterConstants.Configs.RIGHT_MOTOR.apply(rightShootMotor);
        ShooterConstants.Configs.ENCODER.apply(leftShootMotor, leftEncoder);
        ShooterConstants.Configs.ENCODER.apply(rightShootMotor, rightEncoder);
        ShooterConstants.Configs.PID.apply(leftShootMotor, leftShootPID);
        ShooterConstants.Configs.PID.apply(rightShootMotor, rightShootPID);
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("leftTarget", () -> leftTargetSpeed, null);
        builder.addDoubleProperty("rightTarget", () -> rightTargetSpeed, null);
        builder.addBooleanProperty("pidActive", () -> pidActive, null);
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

        double leftDelta = leftSpeed - leftEncoder.getVelocity();
        if (Math.abs(leftDelta) < ShooterConstants.PID_RANGE) {
            leftShootPID.setReference(leftSpeed, ControlType.kVelocity, 0, feedforward.calculate(leftSpeed));
            pidActive = true;
        } else {
            leftShootMotor.set(ShooterConstants.RAMP_SPEED * Math.signum(leftDelta));
            pidActive = false;
        }

        double rightDelta = rightSpeed - rightEncoder.getVelocity();
        if (Math.abs(rightDelta) < ShooterConstants.PID_RANGE) {
            rightShootPID.setReference(rightSpeed, ControlType.kVelocity, 0, feedforward.calculate(rightSpeed));
        } else {
            rightShootMotor.set(ShooterConstants.RAMP_SPEED * Math.signum(rightDelta));
        }
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
        return setSpeed(() -> shooterSpeed).withName("shooter.setSpeed(" + shooterSpeed + ")");
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

    public Command setSpeedWithDist(Supplier<Double> distanceToTarget) {
        return setSpeed(() -> ShooterConstants.DISTANCE_TO_SPEED_MAP.get(distanceToTarget.get())).withName("shooter.setSpeed()");
    }

    /**
     * Spits the note out of the shooter in case it is stuck.
     */
    public Command spitFront() {
        return commandBuilder("shooter.spitFront()")
            .onInitialize(() -> {
                leftShootMotor.set(ShooterConstants.LEFT_SPIT_SPEED_FRONT);
                rightShootMotor.set(ShooterConstants.RIGHT_SPIT_SPEED_FRONT);
            })
            .onEnd(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }

    public Command spitBack() {
        return commandBuilder("shooter.spitBack()")
            .onInitialize(() -> {
                leftShootMotor.set(ShooterConstants.LEFT_SPIT_SPEED_BACK);
                rightShootMotor.set(ShooterConstants.RIGHT_SPIT_SPEED_BACK);
            })
            .onEnd(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }

    /**
     * Runs a SysId quasistatic test.
     * @param direction The direction to run the test in.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("shooter.sysIdQuasistaitc(" + direction + ")");
    }

    /**
     * Runs a SysId dynamic test.
     * @param direction The direction to run the test in.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("shooter.sysIdDynamic(" + direction + ")");
    }
}
