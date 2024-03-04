// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

public class SubsystemGroups {

    public enum Subsystems {
        ROBOT_INDEXER, //Includes note movement pieces of collector, indexer and shooter
        COLLECTOR_PIVOT, //includes both left and right pivot motors
        SHOOTER_PIVOT,
        SHOOTER_FLYWHEEL, //includes both left and right flywheel motors
        CLIMBER_LEFT,
        CLIMBER_RIGHT
    }

    private static Map<Subsystems, RequireableSubsytem> subsystemMap = Map.ofEntries(
        Map.entry(Subsystems.ROBOT_INDEXER, new RequireableSubsytem()),
        Map.entry(Subsystems.COLLECTOR_PIVOT, new RequireableSubsytem()),
        Map.entry(Subsystems.SHOOTER_PIVOT, new RequireableSubsytem()),
        Map.entry(Subsystems.SHOOTER_FLYWHEEL, new RequireableSubsytem()),
        Map.entry(Subsystems.CLIMBER_LEFT, new RequireableSubsytem()),
        Map.entry(Subsystems.CLIMBER_RIGHT, new RequireableSubsytem())
    );

    public static RequireableSubsytem getInstance(Subsystems subsystems) {
        return subsystemMap.get(subsystems);
    }
}
