// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

//This is a stub subsystem
public class Indexer extends SubsystemBase {

    CANSparkMax indexerMotor;
    private DigitalInput shooterBeamBreak;

    /** Creates a new Indexer. */
    public Indexer() {
      shooterBeamBreak = new DigitalInput(Constants.IndexerConstants.SHOOTER_BEAM_BREAK);
        indexerMotor = new CANSparkMax(Constants.IndexerConstants.INDEXER_MOTOR, MotorType.kBrushless);
    }
      public void setSpeedIndexer(double speedMetersPerSecond) {
        indexerMotor.set((speedMetersPerSecond / (Math.PI * (Constants.IndexerConstants.INDEXER_WHEEL_DIAMETER)) * Constants.IndexerConstants.INDEXER_GEAR_RATIO));
      }
      public boolean getBeamBreakSensor() {
        if(shooterBeamBreak.get() == true) {
          return true;
        }
        else {
          return false;
        }
      }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    private static Indexer indexer;

    public static Indexer getInstance() {
      if (indexer == null) {
        indexer = new Indexer();
      }
      return indexer;
    }
}
