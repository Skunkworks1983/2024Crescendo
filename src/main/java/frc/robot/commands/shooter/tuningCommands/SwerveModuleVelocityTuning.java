// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.tuningCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;

public class SwerveModuleVelocityTuning extends Command {
  private Drivebase drivebase;
  double toleranceTicks;
  double timeAtInit;
  double speed = 4.5;
  /** Creates a new SwerveModuleVelocityTuning. */
  public SwerveModuleVelocityTuning() {
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    toleranceTicks = 0;
    timeAtInit = Timer.getFPGATimestamp();
    System.out.println("module tuning command init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Velocity Eror", Math.abs(drivebase.getFieldRelativeSpeeds().vxMetersPerSecond - speed));
    drivebase.setDrive(Units.metersToFeet(speed),0,0,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.setDriveChassisSpeed(new ChassisSpeeds());
    System.out.println("modual tuning command end, time: " + (Timer.getFPGATimestamp() - timeAtInit));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if (Math.abs(drivebase.getFieldRelativeSpeeds().vxMetersPerSecond - speed) < Constants.DRIVEBASE_TUNING_VELOCITY_TOLERANCE) {
      toleranceTicks++;
    } else {
      toleranceTicks = 0;
    }

    if (toleranceTicks >= 5) {
      return true;
    }
    return false;
  }
}
