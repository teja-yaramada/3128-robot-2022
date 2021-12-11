package frc.team3128.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3128.Robot;
import frc.team3128.hardware.NAR_MotorController;
import frc.team3128.hardware.NAR_TalonSRX;
import frc.team3128.hardware.NAR_VictorSPX;
import frc.team3128.hardware.NAR_MotorController.MotorConstants;
import frc.team3128.hardware.NAR_MotorController.MotorControllerType;


public class Intake implements Subsystem {

    private enum IntakeState {
        TOP, BOTTOM;
    }

    private static Intake instance;

    //motors
    private NAR_TalonSRX m_arm_motor;
    private NAR_VictorSPX m_brush_motor_1, m_brush_motor_2, m_intake_motor;

    private DigitalInput m_limitSwitchTop, m_limitSwitchBottom;

    public IntakeState intakeState; 

    public Intake() {
        configMotors();
        configSensors();

        intakeState = IntakeState.TOP;
    }

    public static synchronized Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private void configMotors() {
        m_arm_motor = (NAR_TalonSRX) NAR_MotorController.create(Constants.IntakeConstants.ARM_MOTOR_ID, 
                                                        MotorControllerType.TALON_SRX,
                                                        MotorConstants.Vex775Pro);
        m_brush_motor_1 = (NAR_VictorSPX) NAR_MotorController.create(Constants.IntakeConstants.ARM_MOTOR_ID, 
                                                        MotorControllerType.VICTOR_SPX,
                                                        MotorConstants.Vex775Pro);
        m_brush_motor_2 = (NAR_VictorSPX) NAR_MotorController.create(Constants.IntakeConstants.BRUSH_MOTOR_2_ID, 
                                                        MotorControllerType.VICTOR_SPX,
                                                        MotorConstants.Vex775Pro);
        m_intake_motor = (NAR_VictorSPX) NAR_MotorController.create(Constants.IntakeConstants.INTAKE_MOTOR_ID, 
                                                        MotorControllerType.VICTOR_SPX,
                                                        MotorConstants.Vex775Pro);

        // TODO: What 
        m_arm_motor.setNeutralMode(Constants.IntakeConstants.ARM_NEUTRAL_MODE);
        m_brush_motor_2.set(ControlMode.Follower, Constants.IntakeConstants.BRUSH_MOTOR_1_ID);
    }

    
    private void configSensors() {
        m_limitSwitchTop = new DigitalInput(Constants.IntakeConstants.TOP_LIMIT_SWITCH_ID);
        m_limitSwitchBottom = new DigitalInput(Constants.IntakeConstants.BOTTOM_LIMIT_SWITCH_ID);
    }

    @Override
    public void periodic() {
        if (isBottomTriggered() && intakeState == IntakeState.TOP) {
            intakeState = IntakeState.BOTTOM;
            //stopArm();
        } else if (isTopTriggered() && intakeState == IntakeState.BOTTOM) {
            intakeState = IntakeState.TOP;
            //stopArm();
        }
    }

    public void runIntake() {
        m_intake_motor.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
        m_brush_motor_1.set(ControlMode.PercentOutput, -Constants.IntakeConstants.BRUSH_MOTOR_POWER);
    }

    public void runIntakeOpp() {
        //INTAKE_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
        m_brush_motor_1.set(ControlMode.PercentOutput, Constants.IntakeConstants.BRUSH_MOTOR_POWER);
    }

    public void stopIntake() {
        m_intake_motor.set(ControlMode.PercentOutput, 0);
        m_brush_motor_1.set(ControlMode.PercentOutput, 0);
    }

    public void moveArmUp() {
        if (intakeState == IntakeState.BOTTOM)
            m_arm_motor.set(ControlMode.PercentOutput, -Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void moveArmDown() {
        if (intakeState == IntakeState.TOP)
            m_arm_motor.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void moveArm() {
        if (intakeState == IntakeState.BOTTOM)
            m_arm_motor.set(ControlMode.PercentOutput, -Constants.IntakeConstants.ARM_MOTOR_POWER);
        else {
            m_arm_motor.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER);
        }
    }

    public void moveArmUpAuto() {
        m_arm_motor.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER_AUTO);
    }

    public void stopArm() {
        m_arm_motor.set(ControlMode.PercentOutput, 0);
    }

    public boolean isTopTriggered() {
        return !m_limitSwitchTop.get();
    }

    public boolean isBottomTriggered() {
        return !m_limitSwitchBottom.get();
    }

}

