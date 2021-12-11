package frc.team3128.hardware;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team3128.common.Simulable;

public class NAR_DigitalInput extends DigitalInput implements Simulable{

    private int deviceNumber;
    private SimDevice sensorSim;
    private SimBoolean isOn;

    public NAR_DigitalInput(int deviceNumber){
        super(deviceNumber);
        this.deviceNumber = deviceNumber;
        construct();
    }

    @Override
    public void constructReal() {
        // Done with super
    }

    @Override
    public boolean get(){
        if(sensorSim == null)
            return super.get();
        else
            return isOn.get();
    }

    public void setSensorSim(boolean on){
        if(sensorSim == null)
            // TODO: Call error
            return;
        else
            isOn.set(on);
    }

    @Override
    public void constructFake() {
        sensorSim = SimDevice.create("Sensor ["+deviceNumber+"]", deviceNumber);
        isOn = sensorSim.createBoolean("On", Direction.kBidir, true);
    }

    @Override
    public void updateSimulation(double timeStep) {
        // TODO Auto-generated method stub
    }
    
}
