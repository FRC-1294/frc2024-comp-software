package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LauncherConstants;
import frc.robot.subsystems.Launcher.LauncherMode;

public class LauncherTest extends SubsystemBase{

private double amp_length = 1.5; 
//private double amp_width = 2; 
private double speaker_length = 6.5; 
//private double speaker_width = 3;
private double robot_pos; 
private double robot_shoot_angle;
private LauncherMode mLauncherMode = LauncherMode.SPEAKER;

    public LauncherTest(double pos, double angle){
        robot_pos = pos; 
        robot_shoot_angle = angle; 
    }

    public double calculate_distance() {
        if (mLauncherMode != LauncherMode.SPEAKER){
        return Math.sqrt((amp_length * amp_length) + (robot_pos * robot_pos)); 
        }
    return Math.sqrt((speaker_length * speaker_length) + (robot_pos * robot_pos)); 
    }
    
    public String test_shot() {
      return null; 

    }

} 