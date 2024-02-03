package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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

    public Command gotoAngle(double angle) {
        return home(false)
            .andThen(
                commandBuilder("pivot.gotoAngle()")
                    .onInitialize(() -> {
                        if (angle < PivotConstants.MINIMUM_ANGLE || angle > PivotConstants.MAXIMUM_ANGLE) {
                            DriverStation.reportWarning(
                                "Invalid shooter pivot angle. " +
                                angle +
                                " is not between " +
                                PivotConstants.MINIMUM_ANGLE +
                                " and " +
                                PivotConstants.MAXIMUM_ANGLE,
                                false
                            );
                        } else {
                            pivotPID.setReference(angle, ControlType.kSmartMotion);
                        }
                    })
                    .isFinished(() -> getAtLimit() || Math.abs(pivotEnc.getPosition() - angle) < PivotConstants.CLOSED_LOOP_ERR)
                    .onEnd(interrupted -> {
                        pivotMotor.stopMotor();
                        if (!interrupted) {
                            currentTarget = angle;
                        } else {
                            currentTarget = pivotEnc.getPosition();
                        }
                    })
            );
    }

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
