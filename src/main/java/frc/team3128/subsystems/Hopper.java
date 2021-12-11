package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3128.common.Simulable;
import frc.team3128.hardware.NAR_CANSparkMax;
import frc.team3128.hardware.NAR_DigitalInput;
import frc.team3128.hardware.NAR_MotorController;
import frc.team3128.hardware.NAR_TalonSRX;
import frc.team3128.hardware.NAR_MotorController.MotorConstants;
import frc.team3128.hardware.NAR_MotorController.MotorControllerType;
import net.thefletcher.revrobotics.enums.MotorType;

public class Hopper implements Subsystem, Simulable {

    private static Hopper instance;

    private NAR_TalonSRX m_hopper_1;
    private NAR_CANSparkMax m_hopper_2;

    private NAR_DigitalInput m_bottomSensor, m_topSensor;

    public Hopper() {
        construct();
    }

    public static synchronized Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    public boolean getBottom() {
        return !m_bottomSensor.get();
    }

    public boolean getTop() {
        return !m_topSensor.get();
    }

    // these should be changed to commands 
    public void runHopper(double multiplier) {
        m_hopper_1.set(ControlMode.PercentOutput, Constants.HopperConstants.HOPPER_MOTOR_1_POWER*multiplier);
        m_hopper_2.set(Constants.HopperConstants.HOPPER_MOTOR_2_POWER);
    }

    public void stopHopper() {
        m_hopper_1.set(ControlMode.PercentOutput, 0);
        m_hopper_2.set(0);
    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void constructReal() {
        m_bottomSensor = new NAR_DigitalInput(Constants.HopperConstants.BOTTOM_SENSOR_ID);
        m_topSensor = new NAR_DigitalInput(Constants.HopperConstants.TOP_SENSOR_ID);
        m_hopper_1 = (NAR_TalonSRX) NAR_MotorController.create(Constants.HopperConstants.HOPPER_MOTOR_1_ID, 
                                                    MotorControllerType.TALON_SRX,
                                                    MotorConstants.Vex775Pro);
        m_hopper_2 = new NAR_CANSparkMax(Constants.HopperConstants.HOPPER_MOTOR_2_ID, MotorType.kBrushless);
    }

    @Override
    public void constructFake() {
        
    }

    @Override
    public void updateSimulation(double timeStep) {
        // TODO Auto-generated method stub
        
    }

}
