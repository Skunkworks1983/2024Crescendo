// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

//This is a stub subsystem
public class Indexer extends SubsystemBase {

    CANSparkMax indexerMotor;

    /** Creates a new Indexer. */
    public Indexer() {
        indexerMotor = new CANSparkMax(Constants.IDS.INDEXER_MOTOR, MotorType.kBrushless);
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
