package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Mutable;
import org.team340.robot.Constants.IntakeConstants;
import org.team340.robot.Constants.RobotMap;

/**
 * The Intake subsystem.
 */
public class Intake extends GRRSubsystem {

    private final CANSparkMax pivotLeftMotor = createSparkMax("Deploy Motor", RobotMap.INTAKE_PIVOT_LEFT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax pivotRightMotor = createSparkMax("Deploy Motor", RobotMap.INTAKE_PIVOT_RIGHT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax rollerUpperMotor = createSparkMax("Roller Motor", RobotMap.INTAKE_ROLLER_UPPER_MOTOR, MotorType.kBrushless);
    private final CANSparkMax rollerLowerMotor = createSparkMax("Roller Motor", RobotMap.INTAKE_ROLLER_LOWER_MOTOR, MotorType.kBrushless);

    private final SparkPIDController pivotPID = pivotLeftMotor.getPIDController();
    private final SparkAbsoluteEncoder pivotEncoder = createSparkMaxAbsoluteEncoder("Pivot Encoder", pivotLeftMotor, Type.kDutyCycle);

    public Intake() {
        super("Intake");
        IntakeConstants.PIVOT_LEFT_MOTOR_CONFIG.apply(pivotLeftMotor);
        IntakeConstants.PIVOT_RIGHT_MOTOR_CONFIG.apply(pivotRightMotor);
        IntakeConstants.ROLLER_UPPER_MOTOR_CONFIG.apply(rollerUpperMotor);
        IntakeConstants.ROLLER_LOWER_MOTOR_CONFIG.apply(rollerLowerMotor);

        IntakeConstants.PIVOT_MOTOR_PID_CONFIG.apply(pivotLeftMotor, pivotPID);

        IntakeConstants.PIVOT_ENCODER_CONFIG.apply(pivotLeftMotor, pivotEncoder);
    }

    public Command deploy() {
        Mutable<Boolean> weakPID = new Mutable<>(false);

        return commandBuilder("intake.deploy()")
            .onInitialize(() -> rollerUpperMotor.set(IntakeConstants.DEPLOY_ROLLER_SPEED))
            .onExecute(() -> {
                if (
                    Math2.epsilonEquals(
                        IntakeConstants.DEPLOY_POSITION,
                        pivotLeftMotor.getEncoder().getPosition(),
                        IntakeConstants.DEPLOY_POSITION_TOLERANCE
                    )
                ) weakPID.set(true);
                pivotPID.setReference(IntakeConstants.DEPLOY_POSITION, ControlType.kPosition, weakPID.get() ? 1 : 0);
            })
            .onEnd(() -> {
                rollerUpperMotor.stopMotor();
                pivotLeftMotor.stopMotor();
            });
    }

    public Command retract() {
        return commandBuilder("intake.retract()")
            .onInitialize(() -> rollerUpperMotor.stopMotor())
            .onExecute(() -> {
                pivotPID.setReference(0, ControlType.kPosition, 0);
            });
    }

    public Command spit() {
        Mutable<Boolean> weakPID = new Mutable<>(false);

        return commandBuilder("intake.spit()")
            .onInitialize(() -> rollerUpperMotor.set(IntakeConstants.SPIT_ROLLER_SPEED))
            .onExecute(() -> {
                if (
                    Math2.epsilonEquals(
                        IntakeConstants.DEPLOY_POSITION,
                        pivotLeftMotor.getEncoder().getPosition(),
                        IntakeConstants.DEPLOY_POSITION_TOLERANCE
                    )
                ) weakPID.set(true);

                pivotPID.setReference(IntakeConstants.DEPLOY_POSITION, ControlType.kPosition, weakPID.get() ? 1 : 0);
            })
            .onEnd(() -> {
                rollerUpperMotor.stopMotor();
                pivotLeftMotor.stopMotor();
            });
    }
}