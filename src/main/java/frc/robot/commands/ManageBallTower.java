// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TransportSystem;
import frc.robot.subsystems.TransportSystem.TowerState;

public class ManageBallTower extends CommandBase {
  /** Creates a new ManageBallTower. */
  TransportSystem tower;
  private TowerState curr_state;
  public ManageBallTower(TransportSystem tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    // tower = RobotContainer.m_Transport;
    addRequirements(tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Determine current state
    // low = tower.getLow();
    // mid = tower.getMedium();
    // high = tower.getHigh();
    // tower.determineState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // checks for every case the ball could be in through the censors.
  @Override
  public void execute() {
    curr_state = RobotContainer.m_Transport.getState();
    switch (curr_state) {
      case EMPTY:
        RobotContainer.m_Transport.TowerStop();
        RobotContainer.m_Transport.HopperIn();
        break;
      case READY_2:
      RobotContainer.m_Transport.TowerStop();
      RobotContainer.m_Transport.HopperIn();
        break;
      case READY_3:
      RobotContainer.m_Transport.TowerStop();
      RobotContainer.m_Transport.HopperIn();
        break;
      case LOADED_3:
      RobotContainer.m_Transport.StopTransportSystem();
        break;
      case LOADED_1:
        RobotContainer.m_Transport.TowerUp();
        RobotContainer.m_Transport.HopperStop();
        break;
      case LOADED_2:
      RobotContainer.m_Transport.TowerUp();
      RobotContainer.m_Transport.HopperStop();
        break;
      case UNKNOWN:
        RobotContainer.m_Transport.ReverseTransportSystem();
        break;
      default:
        RobotContainer.m_Transport.StopTransportSystem();
        break;
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // tower.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
