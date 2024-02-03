package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.PivotConstants;
import org.team340.robot.Constants.RobotMap;

public class Pivot extends GRRSubsystem {

    private final CANSparkMax pivotMotor = createSparkMax("Pivot Motor", RobotMap.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder pivotEnc = pivotMotor.getEncoder();
    private final SparkPIDController pivotPID = pivotMotor.getPIDController();

    private final DigitalInput lowerLimit = createDigitalInput("Pivot Lower Limit", RobotMap.PIVOT_LOWER_LIMIT);
    private final DigitalInput upperLimit = createDigitalInput("Pivot Upper Limit", RobotMap.PIVOT_UPPER_LIMIT);

    private boolean hasBeenHomed;
    private double currentTarget;

    public Pivot() {
        super("Pivot");
        PivotConstants.PIVOT_MOTOR_CONFIG.apply(pivotMotor);

        PivotConstants.PIVOT_PID_CONFIG.apply(pivotMotor, pivotPID);

        hasBeenHomed = false;
    }

    private boolean getLowerLimit() {
        return !lowerLimit.get();
    }

    private boolean getUpperLimit() {
        return !upperLimit.get();
    }

    private boolean getAtLimit() {
        return getLowerLimit() || getUpperLimit();
    }

    public boolean isOnTarget() {
        return Math.abs(pivotEnc.getPosition() - currentTarget) < PivotConstants.CLOSED_LOOP_ERR;
    }

    /**
     * Homes the pivot by driving slowly down until reaching limit.
     * @param withOverride If true will ignore {@code hasBeenHomed}.
     */
    public Command home(boolean withOverride) {
        return commandBuilder("pivot.home()")
            .onExecute(() -> {
                if (!hasBeenHomed || withOverride) {
                    pivotMotor.set(PivotConstants.HOMING_SPEED);
                } else {
                    pivotMotor.stopMotor();
                }
            })
            .isFinished(() -> getLowerLimit() || (hasBeenHomed && !withOverride))
            .onEnd(interrupted -> {
                pivotMotor.stopMotor();
                if (getLowerLimit()) {
                    pivotEnc.setPosition(PivotConstants.MINIMUM_ANGLE);
                    hasBeenHomed = true;
                }
            });
    }

    /**
     * Moves position to target angle
     * @param angle The angle that the arm pivots to.
     * @param willFinish Whether or not the command will stop on it's own when it reaches the target angle.
     * @return Returns it's own commandBuilder
     */
    public Command gotoAngle(Supplier<Double> angle, boolean willFinish) {
        return home(false)
            .andThen(
                commandBuilder("pivot.gotoAngle()")
                    .onExecute(() -> {
                        double angleValue = angle.get();
                        if (angleValue < PivotConstants.MINIMUM_ANGLE || angleValue > PivotConstants.MAXIMUM_ANGLE) {
                            DriverStation.reportWarning(
                                "Invalid shooter pivot angle. " +
                                angleValue +
                                " is not between " +
                                PivotConstants.MINIMUM_ANGLE +
                                " and " +
                                PivotConstants.MAXIMUM_ANGLE,
                                false
                            );
                        } else {
                            currentTarget = angle.get();
                            pivotPID.setReference(angleValue, ControlType.kSmartMotion);
                        }
                    })
                    .isFinished(() ->
                        getAtLimit() || (willFinish && Math.abs(pivotEnc.getPosition() - angle.get()) < PivotConstants.CLOSED_LOOP_ERR)
                    )
                    .onEnd(interrupted -> {
                        pivotMotor.stopMotor();
                        if (!interrupted) {
                            currentTarget = angle.get();
                        } else {
                            currentTarget = pivotEnc.getPosition();
                        }
                    })
            );
    }

    /**
     * Maintains current angle
     * @return Returns it's own commandBuilder
     */
    public Command maintainPosition() {
        return commandBuilder("pivot.maintainPosition()")
            .onInitialize(() -> {
                if (hasBeenHomed) {
                    pivotPID.setReference(currentTarget, ControlType.kPosition);
                } else {
                    pivotMotor.stopMotor();
                }
            });
    }
}
