package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    public final String limelightKey = "limelight";
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable(limelightKey);
    public boolean hasTarget;
    public double offsetX; 
    public double area;

    public Vision() {
        hasTarget = false;
        area = 0;
    }

    @Override
    public void periodic() {
        offsetX = limelight.getEntry("tx").getDouble(0);
        area = limelight.getEntry("ta").getDouble(0);
    }
}
