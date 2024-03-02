package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
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

    private final CANSparkFlex leftMotor;
    private final CANSparkFlex rightMotor;
    private final SparkLimitSwitch leftLimit;
    private final SparkLimitSwitch rightLimit;

    /**
     * Create the climber subsystem.
     */
    public Climber() {
        super("Climber");
        leftMotor = createSparkFlex("Left Motor", RobotMap.CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = createSparkFlex("Right Motor", RobotMap.CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);
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

    /**
     * Climbs the chain while balancing.
     * @param robotRoll A supplier that returns the robot's roll in radians.
     */
    public Command climb(Supplier<Double> robotRoll) {
        return commandBuilder("climber.climb()")
            .onExecute(() -> {
                // leftMotor.set(ClimberConstants.CLIMBING_SPEED + (robotRoll.get() * ClimberConstants.BALANCE_COMPENSATION));
                // rightMotor.set(ClimberConstants.CLIMBING_SPEED + (-robotRoll.get() * ClimberConstants.BALANCE_COMPENSATION));
                leftMotor.set(ClimberConstants.CLIMBING_SPEED);
                rightMotor.set(ClimberConstants.CLIMBING_SPEED);
            })
            .onEnd(() -> {
                leftMotor.stopMotor();
                rightMotor.stopMotor();
            });
    }

    /**
     * Drives the climber motors manually.
     * @param speed The speed of the climber as duty cycle.
     */
    public Command driveManual(Supplier<Double> speed) {
        return driveManual(speed, speed);
    }

    /**
     * Drives the climber motors manually.
     * @param leftSpeed The speed of the left climber as duty cycle.
     * @param rightSpeed The speed of the right climber as duty cycle.
     * @return
     */
    public Command driveManual(Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {
        return commandBuilder()
            .onExecute(() -> {
                leftMotor.set(-leftSpeed.get());
                rightMotor.set(-rightSpeed.get());
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
