// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.lang.System.Logger.Level;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Debugger;
import frc.robot.Robot;
import frc.robot.Constants.TowerConstants;
import frc.robot.commands.HopperInCmd;
import frc.robot.commands.WiggleCmd;
import frc.team2363.logger.HelixLogger;


public class TransportSystem extends SubsystemBase {

  public enum TowerState {
    INIT, //Start here
    EMPTY, //all ball sensors indicate no ball - assume ball tower is empty
    LOADED_1, //hopper has pushed 1 ball into lowest spot
    READY_2,  //1st ball is loaded, ready for 2nd
    LOADED_2, //hopper has pushed 2nd ball in
    READY_3, //ready for 3rd ball
    LOADED_3, //all three sensors indicate a ball
    UNKNOWN
  };
  private TowerState prevState = TowerState.INIT;
  private TowerState currentState = TowerState.INIT;

  private static final int kTowerMotor = TowerConstants.TOWER_MOTOR; 
  private static final int kHopperMotor = TowerConstants.HOPPER_MOTOR; 
  private static final int klowSensor = TowerConstants.BOTTOM_SENSOR_DIO;
  private static final int kmidSensor = TowerConstants.MIDDLE_SENSOR_DIO;
  private static final int khighSensor = TowerConstants.TOP_SENSOR_DIO;
  public final VictorSPX TowerVictor;
  public final VictorSPX HopperVictor;
  private DigitalInput levelOne;
  private DigitalInput levelTwo;
  private DigitalInput levelThree;

  private double ballCounter;
  private double up_speed = 0.3;
  private double down_speed = -0.3;
  private double in_speed = 0.3;
  private double out_speed = -0.3;

  /** Creates a new TransportSystem. */
  public TransportSystem() {
    TowerVictor = new WPI_VictorSPX(kTowerMotor);
    HopperVictor = new WPI_VictorSPX(kHopperMotor);
    TowerVictor.configFactoryDefault();
    HopperVictor.configFactoryDefault();

    //VictorSPX doesn't have current limiting capabilities. Might be reason to switch to Talon/SparkMax. 
    //Otherwise implement some current limiting via PDP for checking for ball jams
    TowerVictor.configVoltageCompSaturation(12.0, 0);
    TowerVictor.configOpenloopRamp(0.5, 0);
    HopperVictor.configVoltageCompSaturation(12.0, 0);
    HopperVictor.configOpenloopRamp(0.5, 0);

    levelOne = new DigitalInput(klowSensor);
    levelTwo = new DigitalInput(kmidSensor);
    levelThree = new DigitalInput(khighSensor);
    SmartDashboard.putNumber("Up Speed", up_speed);
    SmartDashboard.putNumber("Down Speed", down_speed);
    SmartDashboard.putNumber("Hopper in Speed", in_speed);
    SmartDashboard.putNumber("Hopper out Speed", out_speed);

    

    //Helixlogger setup
    setupLogs();

    //Default command tries to manage the ball tower states of 0, 1, 2, or 3 balls loaded
    // this.setDefaultCommand(new ManageBallTower());
    this.setDefaultCommand(new HopperInCmd(this));
  }

  public void TowerUp() {
    TowerVictor.set(ControlMode.PercentOutput, up_speed);    
  }

  public void TowerDown() {
    TowerVictor.set(ControlMode.PercentOutput, down_speed);    
  }

  public void TowerStop() {
    TowerVictor.set(ControlMode.PercentOutput, 0.0);  
  }

  public void HopperIn() {
    HopperVictor.set(ControlMode.PercentOutput, in_speed);    
  }

  public void HopperOut() {
    HopperVictor.set(ControlMode.PercentOutput, out_speed);    
  }

  public void HopperStop() {
    HopperVictor.set(ControlMode.PercentOutput, 0.0);  
  }

  public void LoadTransport() {
    TowerUp();
    HopperIn();
  }
  
  public void StopTransportSystem() {
    HopperStop();
    TowerStop();
  }
  public void ReverseTransportSystem() {
    HopperOut();
    TowerDown();
  }

  public void setState(TowerState state) {
    prevState = currentState;
    currentState = state;
  }

  public TowerState getState() {
    return currentState;
  }
  public TowerState getPrevState() {
    return prevState;
  }

  public boolean getLow() {
    return !(levelOne.get());
  }

  public boolean getMedium() {
    return !(levelTwo.get());
  }

  public boolean getHigh() {
    return !(levelThree.get());
  }

  public void incrementBallCounter(){
    ballCounter++;
  }

  public double returnBallCounter(){
    return ballCounter;
  }

  public void resetBallCounter(){

    ballCounter = 0;
  }
  public TowerState determineState() {
    boolean low, mid, high;

    low = getLow();
    mid = getMedium();
    high = getHigh();

    switch(currentState) {
      case INIT:
        //Assume that it is likely to start in either empty or loaded 3 state:
        if(low && mid && high) {
          setState(TowerState.LOADED_3);
        }
        else if (!low && !mid && !high) {
          setState(TowerState.EMPTY);
        }
        else {
          setState(TowerState.UNKNOWN);
        }
        break;
      case EMPTY:
        //In this case - do nothing until a ball is detected low
        if(mid || high) {
          //This is unexpected
          setState(TowerState.UNKNOWN);
        }
        else if (low) {
          //first ball came in
          setState(TowerState.LOADED_1);
        }
        break;
      case LOADED_1:
        //in this case - tower should be lifting up until ball is detected at midpoint
        if(high) {
          //unexpected - this would mean that either 2 sensors are seeing the same ball
          //or something else is causing both the middle and high sensors to read when 
          //only one ball has been loaded
          setState(TowerState.UNKNOWN);
        }        
        else if (!low && mid) {
          //mid is detecting a ball and low is empty 
          setState(TowerState.READY_2);
        }
        break;
      case READY_2:
      //in this case - tower should not be moving until ball enters from hopper
        if (low)
        {
          //now have 2ND ball in the tower
          setState(TowerState.LOADED_2);
        }
        break;
      case LOADED_2:
        //In this case we should be lifting the ball until the ball is detected at mid and high
        if(!low && mid && high)
        {
          setState(TowerState.READY_3);
        }
        break;
      case READY_3:
        //balls should be in top 2 sensors. Tower should not be moving
        if(low && mid && high) {
          setState(TowerState.LOADED_3);
        }
        if(!mid || !high)
        {
          //Lost track of ball in top 2 positions? Maybe reverse a little bit?
          setState(TowerState.UNKNOWN);
        }

        break;
      case LOADED_3:
        //tower should not be moving
        if(!low || !mid || !high)
        {
          //lost track of a ball in a position. Maybe shimmy the tower up and down?
          setState(TowerState.UNKNOWN);
        }
        break;
      case UNKNOWN:
        //take a best guess at determining the state. 
        //maybe have motor run in reverse slowly to allow sensors to change

        //could potentially use prev_state to be smarter about it
        if(!low && !mid && !high) {
          setState(TowerState.EMPTY);
        }
        else if (low && !mid && !high)
        {
          setState(TowerState.LOADED_1);
        }
        else if (!low && mid && !high)
        {
          setState(TowerState.READY_2);
        }
        else if (low && mid && !high)
        {
          setState(TowerState.LOADED_2);
        }
        else if (!low && mid && high)
        {
          setState(TowerState.READY_3);
        }
        else if (low && mid && high)
        {
          setState(TowerState.LOADED_3);
        } 
        else 
        {
          setState(TowerState.UNKNOWN);
        }
        break;
      default:
        currentState = TowerState.UNKNOWN;
      break;
    }
    return currentState;

}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    determineState();
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
  //Set up helixlogger sources here
  private void setupLogs() {
    HelixLogger.getInstance().addSource("TOWER Vel", TowerVictor::getSelectedSensorVelocity);
    HelixLogger.getInstance().addSource("TOWER LOW", levelOne::get);
    HelixLogger.getInstance().addSource("TOWER MID", levelTwo::get);
    HelixLogger.getInstance().addSource("TOWER HIGH",  levelThree::get);
    HelixLogger.getInstance().addSource("TOWER STATE",  currentState::toString);
  }

  public void dashboard() {
    final double up = SmartDashboard.getNumber("Up Speed", up_speed);
    final double down = SmartDashboard.getNumber("Down Speed", down_speed);
    final double in = SmartDashboard.getNumber("Hopper in Speed", in_speed);
    final double out = SmartDashboard.getNumber("Hopper out Speed", out_speed);
    SmartDashboard.putString("Tower State", getState().toString());
    SmartDashboard.putNumber("Tower Speed", TowerVictor.getMotorOutputPercent());
    SmartDashboard.putNumber("Hopper Speed", HopperVictor.getMotorOutputPercent());
    SmartDashboard.putNumber("Ball Counter", ballCounter);

    if (up != up_speed) up_speed = up;
    if (down != down_speed) down_speed = down;
    if (in != in_speed) in_speed = in;
    if (out != out_speed) out_speed = out;
    
  }
  
  public static void printDebug(String msg){
    Debugger.println(msg, Robot._transport, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._transport, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._transport, Debugger.warning4);
  }
}
