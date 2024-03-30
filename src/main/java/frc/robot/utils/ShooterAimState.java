// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class ShooterAimState {

  Translation2d position;
  double flywheelSpeed;
  Rotation2d pivotRotation;

/*
  ShooterAimState(Translation2d position, Rotation2d pivotRotation, double flywheelSpeed){
    this.flywheelSpeed=flywheelSpeed;
    this.position=position;
    this.pivotRotation=pivotRotation;
  }*/
  public ShooterAimState(Translation2d position, double pivotRotation, double flywheelSpeed){
    //this(position, Rotation2d.fromDegrees(pivotRotation),flywheelSpeed);
  }
  public ShooterAimState(){

  }

  public Translation2d getPosition(){return position;}
  public double getFlywheelSpeed(){return flywheelSpeed;}
  public Rotation2d getPivotRotation(){return pivotRotation;}


}
