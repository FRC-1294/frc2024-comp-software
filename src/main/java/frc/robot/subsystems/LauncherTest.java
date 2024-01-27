package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants.LauncherMode;

public class LauncherTest extends SubsystemBase{

private double amp_length = 1.5; 
//private double amp_width = 2; 
private double speaker_length = 6.5; 
//private double speaker_width = 3;
private double robot_pos; 
private LauncherMode mLauncherMode;
private double mvelocity; 
    public LauncherTest(double pos, LauncherMode LauncherMode, double velocity){
        robot_pos = pos; 
        mLauncherMode = LauncherMode; 
        mvelocity = velocity; 
    }


    public Boolean launcher_mode_test(){
        if (mLauncherMode == LauncherMode.OFF){
            if (mvelocity > 0){
                return false; 
            }else{ 
                return true; 
            }
        }else if(mLauncherMode == LauncherMode.AMP && mvelocity == 0 || mLauncherMode == LauncherMode.SPEAKER && mvelocity == 0|| mLauncherMode == LauncherMode.TRAP && mvelocity == 0){
            return false; 
        }
        return true; 
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
