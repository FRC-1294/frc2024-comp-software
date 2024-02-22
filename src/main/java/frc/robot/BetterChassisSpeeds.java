package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class BetterChassisSpeeds extends ChassisSpeeds {
    public double axMetersPerSecond;
    public double ayMetersPerSecond;

    public BetterChassisSpeeds(double vxMPS,double vyMPS, double omegaRPS, double aXMPS, double aYMPS) {
        super(vxMPS, vyMPS, omegaRPS);
        this.axMetersPerSecond = aXMPS;
    }
    
}
