// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransportSystem;

public class TowerUp extends CommandBase {
  private TransportSystem balltower;
  /** Creates a new TowerUp. */
  public TowerUp(TransportSystem tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    balltower = tower;
    addRequirements(balltower);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balltower.incrementBallCounter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (balltower.returnBallCounter() < 3) {
      balltower.TowerUp();
    } else {
      balltower.TowerStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    balltower.TowerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
