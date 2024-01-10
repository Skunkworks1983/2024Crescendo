// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Drivebase extends SubsystemBase {


  public final SwerveModule frontLeft = new SwerveModule(Constants.frontLeftDriveMotorId, Constants.frontLeftTurnMotorId, Constants.frontLeftTurnEncoderId
  ,Constants.frontLeftLocation);
public final SwerveModule frontRight = new SwerveModule(Constants.frontRightDriveMotorId, Constants.frontRightTurnMotorId, Constants.frontRightTurnEncoderId
  ,Constants.frontRightLocation);
public final SwerveModule backLeft = new SwerveModule(Constants.backLeftDriveMotorId, Constants.backLeftTurnMotorId, Constants.backLeftTurnEncoderId
  ,Constants.backLeftLocation);
public final SwerveModule backRight = new SwerveModule(Constants.backRightDriveMotorId, Constants.backRightTurnMotorId, Constants.backRightTurnEncoderId
  ,Constants.backRightLocation);

SwerveModule [] swerveModules = {frontLeft,frontRight,backLeft,backRight};

  /** Creates a new Drivebase. */
  public Drivebase() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
