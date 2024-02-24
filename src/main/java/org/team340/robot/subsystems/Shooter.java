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
import org.team340.robot.Constants;
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

    private double leftTargetSpeed = 0.0;
    private double rightTargetSpeed = 0.0;
    private boolean leftPIDActive = false;
    private boolean rightPIDActive = false;

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

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("atSpeed", this::atSpeed, null);
        builder.addDoubleProperty("leftTarget", () -> leftTargetSpeed, null);
        builder.addDoubleProperty("rightTarget", () -> rightTargetSpeed, null);
        builder.addBooleanProperty("leftPIDActive", () -> leftPIDActive, null);
        builder.addBooleanProperty("rightPIDActive", () -> rightPIDActive, null);
    }

    /**
     * Returns {@code true} if the shooter is at speed.
     */
    public boolean atSpeed() {
        return (
            Math2.epsilonEquals(leftTargetSpeed, leftEncoder.getVelocity(), ShooterConstants.CLOSED_LOOP_ERR) &&
            Math2.epsilonEquals(rightTargetSpeed, rightEncoder.getVelocity(), ShooterConstants.CLOSED_LOOP_ERR)
        );
    }

    /**
     * Applies a specified speed to the shooter.
     * @param speed The speed in RPM to be applied to the left motor, which is scaled by {@link ShooterConstants#RIGHT_PERCENT_OF_LEFT} and applied to the right motor.
     */
    private void applySpeed(double speed) {
        double leftSpeed = speed;
        double rightSpeed = speed * ShooterConstants.RIGHT_PERCENT_OF_LEFT;

        if (speed == 0.0) {
            leftShootMotor.stopMotor();
            rightShootMotor.stopMotor();
            leftPIDActive = false;
            rightPIDActive = false;
        } else {
            double leftDelta = leftSpeed - leftEncoder.getVelocity();
            if (Math.abs(leftDelta) < ShooterConstants.PID_ACTIVE_RANGE) {
                leftShootPID.setReference(leftSpeed, ControlType.kVelocity, 0, feedforward.calculate(leftSpeed));
                leftPIDActive = true;
            } else {
                leftShootMotor.set(ShooterConstants.RAMP_SPEED * Math.signum(leftDelta));
                leftPIDActive = false;
            }

            double rightDelta = rightSpeed - rightEncoder.getVelocity();
            if (Math.abs(rightDelta) < ShooterConstants.PID_ACTIVE_RANGE) {
                rightShootPID.setReference(rightSpeed, ControlType.kVelocity, 0, feedforward.calculate(rightSpeed));
                rightPIDActive = true;
            } else {
                rightShootMotor.set(ShooterConstants.RAMP_SPEED * Math.signum(rightDelta));
                rightPIDActive = false;
            }
        }

        leftTargetSpeed = leftSpeed;
        rightTargetSpeed = rightSpeed;
    }

    /**
     * Uses the {@link ShooterConstants#DISTANCE_MAP distance map} to
     * automatically target the speaker using the supplied distance.
     * Does not end.
     * @param distance A supplier that returns the distance to the speaker in meters.
     */
    public Command targetDistance(Supplier<Double> distance) {
        return targetDistance(distance, 0.0, () -> false).withName("shooter.targetDistance()");
    }

    /**
     * Uses the {@link ShooterConstants#DISTANCE_MAP distance map} to
     * automatically target the speaker using the supplied distance.
     * If the idle supplier is {@code true}, the {@code idleSpeed} is used instead.
     * Does not end.
     * @param distance A supplier that returns the distance to the speaker in meters.
     * @param idleSpeed The speed for the shooter to target in RPM while the idle supplier is {@code true}.
     * @param idle A supplier that while {@code true} will set the shooter to target the specified idle speed.
     */
    public Command targetDistance(Supplier<Double> distance, double idleSpeed, Supplier<Boolean> idle) {
        return setSpeed(() -> idle.get() ? idleSpeed : ShooterConstants.DISTANCE_MAP.get(distance.get()))
            .withName("shooter.targetDistance(idleSpeed: " + idleSpeed + ")");
    }

    /**
     * Sets the speed of the shooter. Does not end.
     * @param speed The speed for the shooter to target in RPM.
     */
    public Command setSpeed(double speed) {
        return setSpeed(() -> speed).withName("shooter.setSpeed(" + speed + ")");
    }

    /**
     * Sets the speed of the shooter. Does not end.
     * @param speed A supplier that returns a speed for the shooter to target in RPM.
     */
    public Command setSpeed(Supplier<Double> speed) {
        return commandBuilder("shooter.setSpeed()")
            .onExecute(() -> applySpeed(speed.get()))
            .onEnd(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }

    /**
     * Sets the shooter to receive a note from the human player.
     */
    public Command intakeHuman() {
        return commandBuilder("shooter.intakeHuman()")
            .onInitialize(() -> {
                leftShootMotor.set(ShooterConstants.INTAKE_HUMAN_SPEED);
                rightShootMotor.set(ShooterConstants.INTAKE_HUMAN_SPEED);
            })
            .onEnd(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }

    /**
     * Spits the note out of the shooter.
     */
    public Command barf() {
        return commandBuilder("shooter.barf()")
            .onInitialize(() -> {
                leftShootMotor.set(ShooterConstants.BARF_SPEED);
                rightShootMotor.set(ShooterConstants.BARF_SPEED);
            })
            .onEnd(() -> {
                leftShootMotor.stopMotor();
                rightShootMotor.stopMotor();
            });
    }

    /**
     * Drives shooter by modifying a moving target RPM.
     * @param rampSpeed The speed to ramp the shooter by in RPM/second.
     */
    public Command driveManual(Supplier<Double> rampSpeed) {
        return setSpeed(() -> leftTargetSpeed + rampSpeed.get() * Constants.PERIOD);
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
