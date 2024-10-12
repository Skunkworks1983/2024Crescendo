// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.CollectNote;
import frc.robot.commands.CollectorStow;
import frc.robot.commands.IntakeNoteToIndexerAuto;
import frc.robot.commands.LowerCollector;
import frc.robot.commands.LowerCollectorAndIntakeToIndexer;
import frc.robot.commands.NoteFloorToShooter;
import frc.robot.commands.SpinUpFlyWheelAndShoot;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TargetPosForAuto;
import frc.robot.commands.WaitDuration;
import frc.robot.commands.shooter.FlywheelSpinup;
import frc.robot.commands.shooter.LoadPieceShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootWhenReady;
import frc.robot.commands.shooter.ShooterToAmp;
import frc.robot.commands.shooter.ShooterToAngle;
import frc.robot.commands.shooter.ShooterToStow;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Targeting.FieldTarget;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector.LimitSwitch;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command swerve;
  private SendableChooser<Command> autoChooser;

  OI oi;
  Drivebase drivebase;
  Shooter shooter;
  Indexer indexer;
  Collector collector;

  @Override
  public void robotInit() {
    oi = OI.getInstance();
    drivebase = Drivebase.getInstance();
    shooter = Shooter.getInstance();
    indexer = Indexer.getInstance();
    collector = Collector.getInstance();
    // enable the driver camera
    CameraServer.startAutomaticCapture();

    // wait times
    NamedCommands.registerCommand("WaitOneSecond", new WaitDuration(1.0));
    NamedCommands.registerCommand("WaitHalfSecond", new WaitDuration(0.5));

    // Collector
    NamedCommands.registerCommand("LowerCollector", new LowerCollector());
    NamedCommands.registerCommand("CollectorStow", new CollectorStow());
    NamedCommands.registerCommand("CollectNote", new CollectNote());

    // Shooter
    NamedCommands.registerCommand("ShootNote", new Shoot());
    NamedCommands.registerCommand("SpinUpFlywheel", new FlywheelSpinup());
    NamedCommands.registerCommand("ShooterToAmp", new ShooterToAmp());
    NamedCommands.registerCommand("ShootWhenReady", new ShootWhenReady());
    NamedCommands.registerCommand("LoadPieceShooter", new LoadPieceShooter());

    // indexer
    NamedCommands.registerCommand("LowerCollectorAndInatake",
        new LowerCollectorAndIntakeToIndexer());
    NamedCommands.registerCommand("IntakeNoteToIndexer", new IntakeNoteToIndexerAuto());

    NamedCommands.registerCommand("NoteFloorToShooter", new NoteFloorToShooter());
    NamedCommands.registerCommand("SpinUpFlyWheelAndShoot", new SpinUpFlyWheelAndShoot());

    NamedCommands.registerCommand("StopRobot", new StopRobot());

    NamedCommands.registerCommand("ShooterPivotW1",
        new ShooterToAngle(Constants.AutoShooting.WNOTE1_ANGLE));
    NamedCommands.registerCommand("ShooterPivotW2",
        new ShooterToAngle(Constants.AutoShooting.WNOTE2_ANGLE));
    NamedCommands.registerCommand("ShooterPivotW3",
        new ShooterToAngle(Constants.AutoShooting.WNOTE3_ANGLE));

    NamedCommands.registerCommand("ShooterToStow", new ShooterToStow());

    NamedCommands.registerCommand("AimAndShootCommand", new AimAndShootCommand());

    NamedCommands.registerCommand("TargetPoseForAuto", new TargetPosForAuto(FieldTarget.SPEAKER));


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Indexer Beambreak", indexer.getBeamBreakSensor());
    SmartDashboard.putBoolean("Shooter Beambreak One", shooter.getShooterIndexerBeambreak1());
    SmartDashboard.putBoolean("Shooter Beambreak Two", shooter.getShooterIndexerBeambreak2());
  }

  @Override
  public void disabledInit() {
    Drivebase.getInstance().setBreakMode(true);
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putBoolean("Indexer Beambreak", indexer.getBeamBreakSensor());
    SmartDashboard.putBoolean("Shooter Beambreak One", shooter.getShooterIndexerBeambreak1());
    SmartDashboard.putBoolean("Shooter Beambreak Two", shooter.getShooterIndexerBeambreak2());
    SmartDashboard.putBoolean("Shooter Limit Forward",
        shooter.getLimitSwitchOutput(Shooter.LimitSwitch.FORWARD_LIMIT_SWITCH));
    SmartDashboard.putBoolean("Shooter Limit Backward",
        shooter.getLimitSwitchOutput(Shooter.LimitSwitch.REVERSE_LIMIT_SWITCH));
    SmartDashboard.putBoolean("Gyro Connected", drivebase.isGyroGood());
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    collector.resetCollectorAngle(Constants.Collector.COLLECTOR_STOW_POS);
    
    // cancels swerve teleop command to make sure it does not interfere with auto
    if (swerve != null) {
      swerve.cancel();
    }

    Command currentAutonomousCommand = autoChooser.getSelected();
    if (currentAutonomousCommand != null) {
      currentAutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Drivebase.getInstance().setBreakMode(false);

    drivebase.setSwerveAsDefaultCommand();
  }

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
