// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import java.util.function.Function;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotDiagnostic.SmartDashboard.SmartDashboardPrintFunction;

public class SmartDashboardOutput {

    public SmartDashboardOutput instance;

    public SmartDashboardOutput(String key, boolean value) {
        instance = new BooleanValue(key, value);
    }

    public SmartDashboardOutput(String key, String value) {
        instance = new StringValue(key, value);
    }

    public SmartDashboardOutput(String key, Sendable value) {
        instance = new SendableValue(key, value);
    }

    private SmartDashboardOutput() {}

    public void put() {}


    class BooleanValue extends SmartDashboardOutput {

        String key;
        boolean value;

        public BooleanValue(String key, boolean value) {
            this.key = key;
            this.value = value;
        }

        @Override
        public void put() {
            SmartDashboard.putBoolean(key, value);
        }
    }

    class StringValue extends SmartDashboardOutput {

        String key;
        String value;

        public StringValue(String key, String value) {
            this.key = key;
            this.value = value;
        }

        @Override
        public void put() {
            SmartDashboard.putString(key, value);
        }
    }

    class SendableValue extends SmartDashboardOutput {

        String key;
        Sendable value;

        public SendableValue(String key, Sendable value) {
            this.value = value;
        }

        @Override
        public void put() {
            SmartDashboard.putData(key, value);
        }
    }
}

