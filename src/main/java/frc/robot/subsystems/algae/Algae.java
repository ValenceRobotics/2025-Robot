package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  public Algae(AlgaeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }

  public void setPivotVoltage(double volts) {
    io.setPivotVoltage(volts);
  }

  public void setShooterVoltage(double volts) {
    io.setShooterVoltage(volts);
  }
}
