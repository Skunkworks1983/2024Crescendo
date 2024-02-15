// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Add your docs here. */
public class SmartPIDControllerCANSparkMax {

    public String name;
    public boolean smart;
    public CANSparkMax motor;
    public double lastKpValue;
    public double lastKiValue;
    public double lastKdValue;
    public double lastKfValue;


    public SmartPIDControllerCANSparkMax(double kp, double ki, double kd, double kf, String name,
            boolean smart, CANSparkMax motor) {

        this.motor = motor;
        this.name = name;
        this.smart = smart;

        lastKpValue = kp;
        lastKiValue = ki;
        lastKdValue = kd;
        lastKfValue = kf;

        motor.getPIDController().setP(kp);
        motor.getPIDController().setI(ki);
        motor.getPIDController().setD(kd);
        motor.getPIDController().setFF(kf);

        SmartDashboard.putNumber(name + " kp Value", kp);
        SmartDashboard.putNumber(name + " ki Value", ki);
        SmartDashboard.putNumber(name + " kd Value", kd);
        SmartDashboard.putNumber(name + " kf Value", kd);
    }

    public void updatePID() {
        //if we pass this test, we are smart, so we can save some bandwith by only grabing the k values once
        if (!smart || !Constants.PIDControllers.SMART_PID_ACTIVE) {
            return;
        }

        double currentKpValue = SmartDashboard.getNumber(name + " kp Value", lastKpValue);
        double currentKiValue = SmartDashboard.getNumber(name + " ki Value", lastKiValue);
        double currentKdValue = SmartDashboard.getNumber(name + " kd Value", lastKdValue);
        double currentKfValue = SmartDashboard.getNumber(name + " kf Value", lastKfValue);

        if (currentKpValue != lastKpValue || currentKiValue != lastKiValue
                || currentKdValue != lastKdValue || currentKfValue != lastKfValue) {

            lastKpValue = currentKpValue;
            lastKiValue = currentKiValue;
            lastKdValue = currentKdValue;
            lastKfValue = currentKfValue;

            motor.getPIDController().setP(lastKpValue);
            motor.getPIDController().setI(lastKiValue);
            motor.getPIDController().setD(lastKdValue);
            motor.getPIDController().setFF(lastKfValue);
        }
    }
}
