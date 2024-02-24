package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
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
    private final SparkLimitSwitch leftLimit;
    private final SparkLimitSwitch rightLimit;

    /**
     * Create the climber subsystem.
     */
    public Climber() {
        super("Climber");
        leftMotor = createSparkMax("Left Motor", RobotMap.CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = createSparkMax("Right Motor", RobotMap.CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);
        leftLimit = leftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        rightLimit = rightMotor.getForwardLimitSwitch(Type.kNormallyOpen);

        ClimberConstants.Configs.MOTOR.apply(leftMotor);
        ClimberConstants.Configs.MOTOR.apply(rightMotor);
        ClimberConstants.Configs.LIMIT.apply(leftMotor, leftLimit);
        ClimberConstants.Configs.LIMIT.apply(rightMotor, rightLimit);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("atLeftLimit", this::getLeftLimit, null);
        builder.addBooleanProperty("atRightLimit", this::getRightLimit, null);
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

    public Command balance(Supplier<Double> robotRoll) {
        return sequence(
            runEnd(
                    () -> {
                        leftMotor.set(ClimberConstants.CLIMBING_SPEED);
                        rightMotor.set(ClimberConstants.CLIMBING_SPEED);
                    },
                    () -> {
                        leftMotor.stopMotor();
                        rightMotor.stopMotor();
                    }
                )
                .until(() -> getLeftLimit() || getRightLimit()),
            runEnd(
                    () -> {
                        if (!getLeftLimit()) leftMotor.set(
                            ClimberConstants.CLIMBING_SPEED * (robotRoll.get() * ClimberConstants.BALANCE_COMPENSATION)
                        );
                        if (!getLeftLimit()) rightMotor.set(
                            ClimberConstants.CLIMBING_SPEED * (-robotRoll.get() * ClimberConstants.BALANCE_COMPENSATION)
                        );
                    },
                    () -> {
                        leftMotor.stopMotor();
                        rightMotor.stopMotor();
                    }
                )
                .until(() -> Math.abs(robotRoll.get()) < ClimberConstants.CLOSED_LOOP_ERR)
        );
    }

    public Command driveManual(Supplier<Double> speed) {
        return driveManual(speed, speed);
    }

    public Command driveManual(Supplier<Double> speed, Supplier<Double> speed2) {
        return commandBuilder()
            .onExecute(() -> {
                double zoom = -speed.get();
                double zoom2 = -speed2.get();
                leftMotor.set(zoom);
                rightMotor.set(zoom2);
            })
            .onEnd(() -> {
                leftMotor.stopMotor();
                rightMotor.stopMotor();
            });
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                // leftMotor.setIdleMode(IdleMode.kCoast);
                // rightMotor.setIdleMode(IdleMode.kCoast);
                leftMotor.stopMotor();
                rightMotor.stopMotor();
            })
            .onEnd(() -> {
                leftMotor.setIdleMode(IdleMode.kBrake);
                rightMotor.setIdleMode(IdleMode.kBrake);
            })
            .ignoringDisable(true)
            .withName("climber.onDisable()");
    }
}
