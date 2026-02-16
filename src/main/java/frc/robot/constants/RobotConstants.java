package frc.robot.constants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotConstants {

    public static BooleanSupplier isRedAlliance = () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    };
    
    public static class Vision {
        public static double LOOKBACK_TIME = 1.0;
    }
}
