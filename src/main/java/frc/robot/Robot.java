package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CANUtil;
import frc.robot.util.LoggedTracer;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("Mode", Constants.getMode().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    // TODO: MA is logging using RLOGServer instead of NT4Publisher. What is better?
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        break;
    }

    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Start AdvantageKit logger
    // TODO: 2025 code has a comment sying this should be after RobotContainer. The template has it
    // before
    AutoLogOutputManager.addObject(new RobotState());
    Logger.start();

    // CTRE Hoot logging
    // do not call the setPath and hoot log will be logged to rio at "/home/lvuser/logs"
    SignalLogger.enableAutoLogging(false);
    // SignalLogger.setPath("//media/sda1/logs");
    // SignalLogger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings.
    robotContainer = new RobotContainer();

    // Warmup pathplanner libraries
    // This must be done after instantiate RobotContainer
    // TODO: These are 2 different commands. Do we need both?
    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();

    // Threads.setCurrentThreadPriority(true, 1);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Refresh all Phoenix signals
    LoggedTracer.reset();
    PhoenixUtil.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // TODO: Learn more about this
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    LoggedTracer.record("CommandScheduler");

    // Return to non-RT thread priority
    Threads.setCurrentThreadPriority(false, 10);

    robotContainer.updateAlerts();

    CANUtil.logStatus();

    // Record cycle time
    LoggedTracer.record("RobotPeriodic");
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // TODO: do we want this. it help during testing, but slows down the trasition after auto
    robotContainer.stopSubsystems();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // Load PathPlanner paths from storage.
    // This will only load before autonomous starts.
    if (DriverStation.isAutonomous() || Constants.getMode() == Constants.Mode.SIM) {
      robotContainer.loadAutonomousPath();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
