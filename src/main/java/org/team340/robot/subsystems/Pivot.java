package org.team340.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.PivotConstants;
import org.team340.robot.Constants.RobotMap;

public class Pivot extends GRRSubsystem {

    private final CANSparkMax pivotMotor = createSparkMax("Pivot Motor", RobotMap.SHOOTER_PIVOT_MOTOR, MotorType.kBrushless);
    private final SparkPIDController pivotPID = pivotMotor.getPIDController();
    private final DigitalInput lowerLimit = new DigitalInput(RobotMap.PIVOT_LOWER_LIMIT);

    private boolean hasBeenHomed;

    public Pivot() {
        super("Pivot");
        PivotConstants.PIVOT_MOTOR_CONFIG.apply(pivotMotor);

        PivotConstants.PIVOT_PID_CONFIG.apply(pivotMotor, pivotPID);
        pivotPID.setSmartMotionMaxVelocity(PivotConstants.MAX_VEL, 0);
        pivotPID.setSmartMotionMaxVelocity(PivotConstants.MAX_VEL, 0);
        pivotPID.setSmartMotionMinOutputVelocity(PivotConstants.MAX_VEL, 0);
        pivotPID.setSmartMotionMaxAccel(PivotConstants.MAX_ACCEL, 0);
        pivotPID.setSmartMotionAllowedClosedLoopError(PivotConstants.CLOSED_LOOP_ERR, 0);

        hasBeenHomed = false;
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
            .isFinished(() -> lowerLimit.get() || (hasBeenHomed && !withOverride))
            .onEnd(interrupted -> {
                pivotMotor.stopMotor();
                hasBeenHomed = !interrupted;
            });
    }
}
