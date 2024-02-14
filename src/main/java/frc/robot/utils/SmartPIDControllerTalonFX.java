// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class SmartPIDControllerTalonFX {

    public String name;
    public boolean smart;
    public TalonFX motor;
    public double lastKpValue;
    public double lastKiValue;
    public double lastKdValue;
    public double lastKfValue;


    public SmartPIDControllerTalonFX(double kp, double ki, double kd, double kf, String name,
            boolean smart, TalonFX motor) {

        this.motor = motor;
        this.name = name;
        this.smart = smart;

        lastKpValue = kp;
        lastKiValue = ki;
        lastKdValue = kd;
        lastKfValue = kf;

        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kP = lastKpValue;
        slot0Configs.kI = lastKiValue;
        slot0Configs.kD = lastKdValue;
        slot0Configs.kV = lastKfValue;

        motor.getConfigurator().apply(slot0Configs);

        SmartDashboard.putNumber(name + " kp Value", kp);
        SmartDashboard.putNumber(name + " ki Value", ki);
        SmartDashboard.putNumber(name + " kd Value", kd);
        SmartDashboard.putNumber(name + " kf Value", kd);
    }

    public void updatePID() {
        if (smart && Constants.PIDControllers.SMART_PID_ACTIVE
                && (SmartDashboard.getNumber(name + " kp Value", lastKpValue) != lastKpValue
                        || SmartDashboard.getNumber(name + " ki Value", lastKiValue) != lastKiValue
                        || SmartDashboard.getNumber(name + " kd Value", lastKdValue) != lastKdValue
                        || SmartDashboard.getNumber(name + " kf Value",
                                lastKfValue) != lastKfValue)) {

            lastKpValue = SmartDashboard.getNumber(name + " kp Value", lastKpValue);
            lastKiValue = SmartDashboard.getNumber(name + " ki Value", lastKiValue);
            lastKdValue = SmartDashboard.getNumber(name + " kd Value", lastKdValue);
            lastKfValue = SmartDashboard.getNumber(name + " kf Value", lastKfValue);

            Slot0Configs slot0Configs = new Slot0Configs();

            slot0Configs.kP = lastKpValue;
            slot0Configs.kI = lastKiValue;
            slot0Configs.kD = lastKdValue;
            slot0Configs.kV = lastKfValue;

            motor.getConfigurator().apply(slot0Configs);
        }

        if (smart && Constants.PIDControllers.SMART_PID_ACTIVE) {
            SmartDashboard.putNumber(name + " Error",
                    motor.getClosedLoopError().getValueAsDouble());
        }
    }
}
