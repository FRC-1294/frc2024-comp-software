package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private int _id;
    private CANSparkMax _motor;
    private DigitalInput _beambreak;


    /**
     * Creates the Intake Subsystem
     * @param id The ID of the Spark Max for the Intake motor
     * @param beambreakID the DIO port the Intake beam break reciever is connected to
     */
    public Intake(int id, int beambreakID) {
        _id = id;
        _motor = new CANSparkMax(_id, MotorType.kBrushless);
        _beambreak = new DigitalInput(beambreakID);
    }

    /**
     * @return returns the CANSparkMax ID
     */
    public int getID() {
        return _id;
    }

    /**
     * @return returns intake motor
     */
    public CANSparkMax getMotor() {
        return _motor;
    }

    /**
     * @return returns the DIO port for the beambreak sensors
     */
    public int getBeamBreakID() {
        return _beambreak.getChannel();
    }

    /**
     * @return returns the beambreak object
     */
    public DigitalInput getBeamBreak() {
        return _beambreak;
    }

    /**
     * Sets the motor to a certain speed with no PID. 
     * <p> Run with negative speed to intake
     * @param speed from -1.0 to 1.0
     */
    public void runMotorRaw(double speed) {
        _motor.set(speed);
    }

    /**
     * Stops the motor by seting motor speed to 0.0 with no PID
     */
    public void stopMotor() {
        _motor.set(0.0);
    }

    /**
     * @return returns true if there is a game piece in the intake
     */
    public boolean gamePieceInIntake() {
        return !_beambreak.get();
    }
}
