package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.team3128.hardware.NAR_Motor;

public class NAR_TalonSRX extends NAR_Motor<WPI_TalonSRX>{



    //Lazy Logic Variables
    protected double prevValue = 0;
	protected ControlMode prevControlMode = ControlMode.Disabled;

    NAR_TalonSRX(int deviceNumber) {
		super(deviceNumber);
	}

	@Override
	public void constructReal() {
		this.motorController = new WPI_TalonSRX(deviceNumber);
		motorController.enableVoltageCompensation(true);
		motorController.configVoltageCompSaturation(12, 10);
	}

	@Override
	public void set(ControlMode controlMode, double value) {
		if (value != prevValue || controlMode != prevControlMode) {
			motorController.set(controlMode, value);
			prevValue = value;
		}
	}

	@Override
	public void set(double value) {
		motorController.set(prevControlMode, value);
	}

    @Override
	public double getSetpoint() {
		return prevValue;
	}

	@Override
	public void setNeutralMode(NAR_NeutralMode mode){
		switch(mode) {
			case BRAKE:
				motorController.setNeutralMode(NeutralMode.Brake);	
				break;
			case COAST:
				motorController.setNeutralMode(NeutralMode.Coast);
				break;
			default:
				break;
		}
	}

	@Override
	public double getEncoderPosition(){
		if(simEncoder == null)
			return motorController.getSelectedSensorPosition();
		else
			return simPos.get();
	}

	@Override
	public double getEncoderVelocity(){
		if(simEncoder == null)
			return motorController.getSelectedSensorVelocity();
		else
			return simVel.get();
	}

	@Override
	public void setEncoderPosition(double pos) {
		if(simEncoder == null)
			motorController.setSelectedSensorPosition(pos);
		else
			simPos.set(pos);
	}
}