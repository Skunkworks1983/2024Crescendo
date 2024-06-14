// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import java.util.HashMap;
import java.util.Map;

public class DiagnosticResults {

    static Map<String, Runnable> diagnosticResults = new HashMap<String, Runnable>();

    public static void addResults(Map<String, Runnable> newResults) {
        for (Map.Entry<String, Runnable> entry : newResults.entrySet()) {
            diagnosticResults.put(entry.getKey(), entry.getValue());
        }
    }

    public static Map<String, Runnable> getResults() {
        return diagnosticResults;
    }
}
