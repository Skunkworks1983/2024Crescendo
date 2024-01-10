// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SwerveModule extends SubsystemBase {


    private final PIDController drivePIDController = new PIDController(.12, 0, .0000);
    private final PIDController turnPIDController = new PIDController(.01, 0, .00);
    private Translation2d translation;

  /** Creates a new SwerveModule. */
public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderChannelA,
            Translation2d translation) {
        this.translation = translation;
        //driveMotor = new TalonFX(driveMotorChannel);
        //turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
            }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
