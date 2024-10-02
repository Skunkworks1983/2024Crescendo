// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.ClimberConstants.ClimbModule;
import frc.robot.constants.Constants.ClimberConstants.ClimberSlotConfigs;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ClimbWithGyro extends Command {
  /** Creates a new ClimbWithGyro. */
  Climber climber;
  Drivebase drivebase;
  double leftOutput, rightOutput;

  public ClimbWithGyro() {
    climber = Climber.getInstance();
    drivebase = Drivebase.getInstance();
    leftOutput = 0;
    rightOutput = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    addRequirements(SubsystemGroups.getInstance(Subsystems.CLIMBER_RIGHT),
        SubsystemGroups.getInstance(Subsystems.CLIMBER_LEFT));

    System.out.println("Climb With Gyro Command Initialize");
    SmartDashboard.putBoolean("ClimbWithGyro", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roll = drivebase.getRoll();

    leftOutput =
        ClimberConstants.BASE_PULL_SPEED - (roll / ClimberConstants.ROLL_DEGREES_TO_OUTPUT);
    rightOutput =
        ClimberConstants.BASE_PULL_SPEED + (roll / ClimberConstants.ROLL_DEGREES_TO_OUTPUT);

    SmartDashboard.putNumber("Left Climber Output", leftOutput);
    SmartDashboard.putNumber("Right Climber Output", rightOutput);

    climber.setClimberOutput(ClimbModule.LEFT, leftOutput);
    climber.setClimberOutput(ClimbModule.RIGHT, rightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Climb With Gyro Command End");
    climber.setClimberOutput(ClimbModule.LEFT, -0.02);
    climber.setClimberOutput(ClimbModule.RIGHT, -0.02);
    climber.setBrakeMode(ClimbModule.LEFT);
    climber.setBrakeMode(ClimbModule.RIGHT);
    climber.applySlotConfig(ClimbModule.LEFT, ClimberSlotConfigs.SLOT_1);
    climber.applySlotConfig(ClimbModule.RIGHT, ClimberSlotConfigs.SLOT_1);
    SmartDashboard.putBoolean("ClimbWithGyro", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Checks if one of the climbers is at the minimun position, if yes, it ends the
    // command
    if (climber.getClimberPostition(ClimbModule.LEFT) <= ClimberConstants.MIN_POSITION
        || climber.getClimberPostition(ClimbModule.RIGHT) <= ClimberConstants.MIN_POSITION) {
          
          System.out.println("left climber pos " + climber.getClimberPostition(ClimbModule.LEFT));
    System.out.println("right climber pos " + climber.getClimberPostition(ClimbModule.RIGHT));
    System.out.println("interupted");
      return true;

      
    }

    System.out.println("left climber pos " + climber.getClimberPostition(ClimbModule.LEFT));
    System.out.println("right climber pos " + climber.getClimberPostition(ClimbModule.RIGHT));

    return false;
  }
}
