package frc.states;

public interface MechState {
    void setFlywheelOff();
    void setSpeakerSP();
    void setAmpSP();
    void setTrapSP();
    void intake();
    void index();
    void controlWrist();
    void controlElevator();
    void setElevatorSPtoStage();
    void setElevatorSPtoBase();
    void resetEncoders();
}
