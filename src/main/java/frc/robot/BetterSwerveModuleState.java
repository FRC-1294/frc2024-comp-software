package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class BetterSwerveModuleState extends SwerveModuleState{
    public double accelerationMetersPerSecond;
    public double angularVelocityMetersPerSecond;

    public BetterSwerveModuleState(double speedMetersPerSecond, Rotation2d angle, double accelerationMetersPerSecond, double angularVelocityMetersPerSecond) {
        super(speedMetersPerSecond, angle);
        this.accelerationMetersPerSecond = accelerationMetersPerSecond;
        this.angularVelocityMetersPerSecond = angularVelocityMetersPerSecond;
    }
    
}
