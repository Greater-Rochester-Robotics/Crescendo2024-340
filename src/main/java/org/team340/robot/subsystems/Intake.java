package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
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

    private final CANSparkMax deployMotor = createSparkMax("Deploy Motor", RobotMap.INTAKE_DEPLOY_MOTOR, MotorType.kBrushless);
    private final CANSparkMax rollerMotor = createSparkMax("Roller Motor", RobotMap.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);

    private final SparkPIDController deployPID = deployMotor.getPIDController();

    public Intake() {
        super("Intake");
        deployPID.setP(IntakeConstants.INTAKE_DEPLOY_STRONG_PID.p(), 0);
        deployPID.setI(IntakeConstants.INTAKE_DEPLOY_STRONG_PID.i(), 0);
        deployPID.setD(IntakeConstants.INTAKE_DEPLOY_STRONG_PID.d(), 0);

        deployPID.setP(IntakeConstants.INTAKE_DEPLOY_WEAK_PID.p(), 1);
        deployPID.setI(IntakeConstants.INTAKE_DEPLOY_WEAK_PID.i(), 1);
        deployPID.setD(IntakeConstants.INTAKE_DEPLOY_WEAK_PID.d(), 1);
    }

    public Command deploy() {
        Mutable<Boolean> weakPID = new Mutable<>(false);

        return commandBuilder("intake.deploy()")
            .onInitialize(() -> rollerMotor.set(IntakeConstants.INTAKE_DEPLOY_ROLLER_SPEED))
            .onExecute(() -> {
                if (
                    Math2.epsilonEquals(
                        IntakeConstants.INTAKE_DEPLOY_POSITION,
                        deployMotor.getEncoder().getPosition(),
                        IntakeConstants.INTAKE_DEPLOY_POSITION_TOLERANCE
                    )
                ) weakPID.set(true);
                deployPID.setReference(IntakeConstants.INTAKE_DEPLOY_POSITION, ControlType.kPosition, weakPID.get() ? 1 : 0);
            })
            .onEnd(() -> {
                rollerMotor.stopMotor();
                deployMotor.stopMotor();
            });
    }

    public Command retract() {
        return commandBuilder("intake.retract()")
            .onInitialize(() -> rollerMotor.stopMotor())
            .onExecute(() -> {
                deployPID.setReference(0, ControlType.kPosition, 0);
            });
    }

    public Command spit() {
        Mutable<Boolean> weakPID = new Mutable<>(false);

        return commandBuilder("intake.spit()")
            .onInitialize(() -> rollerMotor.set(IntakeConstants.INTAKE_SPIT_ROLLER_SPEED))
            .onExecute(() -> {
                if (
                    Math2.epsilonEquals(
                        IntakeConstants.INTAKE_DEPLOY_POSITION,
                        deployMotor.getEncoder().getPosition(),
                        IntakeConstants.INTAKE_DEPLOY_POSITION_TOLERANCE
                    )
                ) weakPID.set(true);

                deployPID.setReference(IntakeConstants.INTAKE_DEPLOY_POSITION, ControlType.kPosition, weakPID.get() ? 1 : 0);
            })
            .onEnd(() -> {
                rollerMotor.stopMotor();
                deployMotor.stopMotor();
            });
    }
}
