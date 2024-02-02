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
  double defaultSetpoint = 0;
  double angularPos = 0;
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
      defaultSetpoint = -drivebase.getGyroAngle();
      timeAtLastInput = timer.getFPGATimestamp();
    }
    else if(oi.getTargetingButton()) {

      angularPos = Units.radiansToDegrees(-Math.atan2((Constants.TARGETING_POSITION_Y - drivebase.getRobotPose().getY()), (Constants.TARGETING_POSITION_X - drivebase.getRobotPose().getX())));
      defaultSetpoint = -drivebase.getGyroAngle();
      timeAtLastInput = timer.getFPGATimestamp();
    }
    else{
      if(timeAtLastInput - timer.getFPGATimestamp() < 1){
        defaultSetpoint = -drivebase.getGyroAngle();
      }
      angularPos = defaultSetpoint;
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
      drivebase.setHeadingController(angularPos);
      drivebase.setDriveTurnPos(
        MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        true
      );
    }

    /*
    if(Math.abs(oi.getRightX())<=Constants.X_JOY_DEADBAND && !inJoystickDeadzone){
      drivebase.setHeadingController(-drivebase.getGyroAngle());
      inJoystickDeadzone = true;
    }
    if(Math.abs(oi.getRightX())>Constants.X_JOY_DEADBAND && inJoystickDeadzone){
      inJoystickDeadzone = false;
    }

    if(!inJoystickDeadzone){

      drivebase.setDrive(
        MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getRightX(), Constants.ROT_JOY_DEADBAND) * Constants.OI_TURN_SPEED_RATIO,
        true
      );
    }
    else{
      if(oi.getTargetingButton()) {

        double targetingAngle = -Math.atan2((Constants.TARGETING_POSITION_Y - drivebase.getRobotPose().getY()), (Constants.TARGETING_POSITION_X - drivebase.getRobotPose().getX()));
        drivebase.setHeadingController(Math.toDegrees(targetingAngle));
        System.out.println("Turning to: "+targetingAngle);
      }

      drivebase.setDriveTurnPos(
        MathUtil.applyDeadband(oi.getLeftY(), Constants.X_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        MathUtil.applyDeadband(oi.getLeftX(), Constants.Y_JOY_DEADBAND) * Constants.OI_DRIVE_SPEED_RATIO,
        true
      );
    }
      */
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