package frc.robot;

import frc.robot.lib.controls.XBoxController;


public class IO { 
    private static IO instance; 
    private XBoxController driverController;
    private XBoxController manipulatorController;
    
    private IO() {
        driverController = new XBoxController(0);
        manipulatorController = new XBoxController(1);
        
        //coralStationProximity = new Trigger(SubsystemsInst.getInst().drivetrain.nearRedCoral());
    }
public static IO getInstance() {
    if(instance == null) instance = new IO();

    return instance;
}

public void teleopInit(){
   
}

public XBoxController getDriverController(){
    return driverController;
}
public XBoxController getManipulatorController(){
    return manipulatorController;

}
}