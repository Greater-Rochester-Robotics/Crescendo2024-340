package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.ClimberConstants;
import org.team340.robot.Constants.RobotMap;

/**
 * The climber subsystem. Homed using limit switches, controlled
 * with PID to climb on the chain in two stages.
 */
public class Climber extends GRRSubsystem {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final SparkLimitSwitch leftLimit;
    private final SparkLimitSwitch rightLimit;
    private final SparkPIDController leftPID;
    private final SparkPIDController rightPID;

    private boolean isHomed = false;
    private double target = 0.0;

    /**
     * Create the climber subsystem.
     */
    public Climber() {
        super("Climber");
        leftMotor = createSparkMax("Left Motor", RobotMap.CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = createSparkMax("Right Motor", RobotMap.CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftLimit = leftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        rightLimit = rightMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        leftPID = leftMotor.getPIDController();
        rightPID = rightMotor.getPIDController();

        ClimberConstants.Configs.MOTOR.apply(leftMotor);
        ClimberConstants.Configs.MOTOR.apply(rightMotor);
        ClimberConstants.Configs.ENCODER.apply(leftMotor, leftEncoder);
        ClimberConstants.Configs.ENCODER.apply(rightMotor, rightEncoder);
        ClimberConstants.Configs.LIMIT.apply(leftMotor, leftLimit);
        ClimberConstants.Configs.LIMIT.apply(rightMotor, rightLimit);
        ClimberConstants.Configs.PID.apply(leftMotor, leftPID);
        ClimberConstants.Configs.PID.apply(rightMotor, rightPID);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("atLeftLimit", this::getLeftLimit, null);
        builder.addBooleanProperty("atRightLimit", this::getRightLimit, null);
        builder.addBooleanProperty("isHomed", () -> isHomed, null);
        builder.addDoubleProperty("target", () -> target, null);
    }

    /**
     * Returns {@code true} if the left limit is pressed.
     */
    private boolean getLeftLimit() {
        return leftLimit.isPressed();
    }

    /**
     * Returns {@code true} if the right limit is pressed.
     */
    private boolean getRightLimit() {
        return rightLimit.isPressed();
    }

    /**
     * Homes the climber arms using their limit switch. Doesn't home if the climber
     * has already been homed, unless {@code withOverride} is {@code true}.
     * @param withOverride If {@code true}, ignores {@link #isHomed}.
     */
    public Command home(boolean withOverride) {
        return either(
            commandBuilder()
                .onInitialize(() -> target = ClimberConstants.MAX_POS)
                .onExecute(() -> {
                    if (!getLeftLimit()) {
                        leftMotor.set(ClimberConstants.ZEROING_SPEED);
                    } else {
                        leftMotor.stopMotor();
                    }

                    if (!getRightLimit()) {
                        rightMotor.set(ClimberConstants.ZEROING_SPEED);
                    } else {
                        rightMotor.stopMotor();
                    }
                })
                .isFinished(() -> getLeftLimit() && getRightLimit())
                .onEnd(interrupted -> {
                    rightMotor.stopMotor();
                    leftMotor.stopMotor();

                    if (!interrupted) {
                        rightEncoder.setPosition(ClimberConstants.MAX_POS);
                        leftEncoder.setPosition(ClimberConstants.MAX_POS);
                        isHomed = true;
                    }
                }),
            none(),
            () -> withOverride || !isHomed
        )
            .withName("climber.home(" + withOverride + ")");
    }

    /**
     * Moves the climber arms to a supplied position. Homes if the climber has not yet been homed.
     * @param position This is the position the arms will be set to, must be between the
     * {@link ClimberConstants#MIN_POS minimum} and {@link ClimberConstants#MAX_POS maximum} positions.
     */
    public Command toPosition(double position) {
        return either(
            sequence(
                home(false),
                commandBuilder()
                    .onInitialize(() -> target = position)
                    .onExecute(() -> {
                        double difference = leftEncoder.getPosition() - rightEncoder.getPosition();
                        leftPID.setReference(position - (difference * ClimberConstants.BALANCE_COMPENSATION), ControlType.kPosition);
                        rightPID.setReference(position + (difference * ClimberConstants.BALANCE_COMPENSATION), ControlType.kPosition);
                    })
                    .onEnd(() -> {
                        leftMotor.stopMotor();
                        rightMotor.stopMotor();
                    })
            ),
            runOnce(() -> DriverStation.reportWarning("The climber cannot be set to " + position + " (out of range).", false)),
            () -> position >= ClimberConstants.MIN_POS && position <= ClimberConstants.MAX_POS
        )
            .withName("climber.toPosition(" + position + ")");
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                leftMotor.setIdleMode(IdleMode.kCoast);
                rightMotor.setIdleMode(IdleMode.kCoast);
            })
            .onEnd(() -> {
                leftMotor.setIdleMode(IdleMode.kBrake);
                rightMotor.setIdleMode(IdleMode.kBrake);
            })
            .ignoringDisable(true)
            .withName("climber.onDisable()");
    }
}
