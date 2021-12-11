package frc.team3128.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.team3128.hardware.NAR_MotorController;

public class NAR_VictorSPX extends NAR_MotorController<WPI_VictorSPX> {

  //Lazy Logic Variables
  protected double prevValue = 0;
  protected ControlMode prevControlMode = ControlMode.Disabled;

    public NAR_VictorSPX(int deviceNumber) {
		super(deviceNumber);
	}

	@Override
	public void constructReal() {
		this.motorController = new WPI_VictorSPX(deviceNumber);
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
			// return motorController.getSelectedSensorPosition();
			// TODO Call error
			return 0;
		else
			return simPos.get();
	}

	@Override
	public double getEncoderVelocity(){
		if(simEncoder == null)
			// return motorController.getSelectedSensorVelocity();
			// TODO Call error
			return 0;
		else
			return simVel.get();
	}

	@Override
	public void setEncoderPosition(double pos) {
		if(simEncoder == null)
			// return motorController.getSelectedSensorVelocity();
			// TODO Call error
			return;
		else
			simPos.set(pos);
	}

	@Override
	public void follow(NAR_MotorController<WPI_VictorSPX> motor) {
		motorController.follow(motor.getMotorController());
	}
}