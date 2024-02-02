// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;


public class SwerveTeleop extends Command {
  
  Drivebase drivebase;
  OI oi; 
  double setpointHeadingControl = 0; //this is updated to the robots current angle when using the targeting button or not in the turn joystick deadzone. Used for heading correction when not using the targeting button and in the turn joystick deadzone
  double desiredHeadingSetpoint = 0; //parts of the code set this variable, and then it is used to tell the drive command that turns to a certan angle instead of turning at a speed where to turn to
  Timer timer;
  double timeAtLastInput;


  public SwerveTeleop(Drivebase drivebase, OI oi) {
    timer = new Timer();
    timeAtLastInput = timer.getFPGATimestamp();
    this.drivebase = drivebase;
    this.oi = oi;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double velocity = 0;

    if(Math.abs(oi.getRightX())>Constants.ROT_JOY_DEADBAND) {

      velocity = oi.getRightX() * Constants.OI_TURN_SPEED_RATIO;
      setpointHeadingControl = -drivebase.getGyroAngle();
      timeAtLastInput = timer.getFPGATimestamp();
    }
    else if(oi.getTargetingButton()) {
      //calculates our current position on the field and where we are targeting to and figures out the angle to point at
      desiredHeadingSetpoint = Units.radiansToDegrees(-Math.atan2((Constants.TARGETING_POSITION_Y - drivebase.getRobotPose().getY()), (Constants.TARGETING_POSITION_X - drivebase.getRobotPose().getX())));
      setpointHeadingControl = -drivebase.getGyroAngle();
      timeAtLastInput = timer.getFPGATimestamp();
    }
    else{
      //waits a second to allow for extra turn momentum to dissipate
      if(timeAtLastInput - timer.getFPGATimestamp() < Constants.TIME_UNTIL_HEADING_CONTROL){
        setpointHeadingControl = -drivebase.getGyroAngle();
      desiredHeadingSetpoint = setpointHeadingControl;
      }
    }
    
    if(Math.abs(velocity) > 0)
    {
      drivebase.setDrive(
        MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        velocity,
        true
      );
    }
    else{
      drivebase.setHeadingController(desiredHeadingSetpoint);
      drivebase.setDriveTurnPos(
        MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        true
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}