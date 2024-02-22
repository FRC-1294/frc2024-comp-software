package frc.robot;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Arrays;

public class SwerveSecondOrderKinematics {

    private final SimpleMatrix m_inverseKinematics;
  
    private final int m_numModules;
    private final Translation2d[] m_modules;
    private Rotation2d[] m_moduleHeadings;
    private Translation2d m_prevCoR = new Translation2d();


    public SwerveSecondOrderKinematics(Translation2d... moduleTranslationsMeters) {
    if (moduleTranslationsMeters.length < 2) {
      throw new IllegalArgumentException("A swerve drive requires at least two modules");
    }
    m_numModules = moduleTranslationsMeters.length;
    m_modules = Arrays.copyOf(moduleTranslationsMeters, m_numModules);
    m_moduleHeadings = new Rotation2d[m_numModules];
    Arrays.fill(m_moduleHeadings, new Rotation2d());
    m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

    for (int i = 0; i < m_numModules; i++) {
      m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getY());
      m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].getX());
    }

    MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
  }

  public BetterSwerveModuleState[] toSwerveModuleStates(
      BetterChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
   /* var moduleStates = new SwerveModuleState[m_numModules];

    if (chassisSpeeds.vxMetersPerSecond == 0.0
        && chassisSpeeds.vyMetersPerSecond == 0.0
        && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
      for (int i = 0; i < m_numModules; i++) {
        moduleStates[i] = new BetterSwerveModuleState(0.0, m_moduleHeadings[i]);
      }

      return moduleStates;
    }

    if (!centerOfRotationMeters.equals(m_prevCoR)) {
      for (int i = 0; i < m_numModules; i++) {
        m_inverseKinematics.setRow(
            i * 2 + 0,
            0, /* Start Data
            1,
            0,
            -m_modules[i].getY() + centerOfRotationMeters.getY());
        m_inverseKinematics.setRow(
            i * 2 + 1,
            0, /* Start Data 
            0,
            1,
            +m_modules[i].getX() - centerOfRotationMeters.getX());
      }
      m_prevCoR = centerOfRotationMeters;
    }

    var chassisSpeedsVector = new SimpleMatrix(3, 1);
    var chassis
    chassisSpeedsVector.setColumn(
        0,
        0,
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);

    var moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);

    for (int i = 0; i < m_numModules; i++) {
      double x = moduleStatesMatrix.get(i * 2, 0);
      double y = moduleStatesMatrix.get(i * 2 + 1, 0);

      double speed = Math.hypot(x, y);
      Rotation2d angle = new Rotation2d(x, y);

      moduleStates[i] = new SwerveModuleState(speed, angle);
      m_moduleHeadings[i] = angle;
    } */
    //turn a, v and θ requests
    
    
    
    var chassisSpeedsVector = new SimpleMatrix(0, 1);
  }
}
