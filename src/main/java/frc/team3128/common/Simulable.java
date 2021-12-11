package frc.team3128.common;

import edu.wpi.first.wpilibj.RobotBase;

public interface Simulable {
    public default void construct(){
        // We will always need the real code for any simulated class
        constructReal();
        if(RobotBase.isSimulation()){
            constructFake();
        }
    }

    public abstract void constructReal();
    public abstract void constructFake();
    public abstract void updateSimulation(double timeStep);
}