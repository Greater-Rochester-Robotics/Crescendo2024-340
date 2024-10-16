package org.team340.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team340.lib.dashboard.GRRDashboard;
import org.team340.lib.util.Profiler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
public final class Robot extends TimedRobot {

    private final RobotContainer robotContainer;
    private Command autoCommand;

    public Robot() {
        super(Constants.kPeriod);
        DriverStation.silenceJoystickConnectionWarning(true);

        // Start logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        Epilogue.configure(config -> {
            config.root = "Telemetry";
        });

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Profiler.start("RobotPeriodic");
        Profiler.run("CommandScheduler", () -> CommandScheduler.getInstance().run());
        Profiler.run("Epilogue", () -> Epilogue.update(this));
        Profiler.run("GRRDashboard", GRRDashboard::update);
        Profiler.end();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autoCommand = GRRDashboard.getSelectedAuto();
        CommandScheduler.getInstance().schedule(autoCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        if (autoCommand != null) autoCommand.cancel();
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
