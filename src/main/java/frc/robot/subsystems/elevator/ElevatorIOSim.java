package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotState.ElevatorState;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim m_elevatorSim =
      new ElevatorSim(
          ElevatorConstants.m_elevatorMotors,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMassKg,
          ElevatorConstants.kElevatorDrumRadiusMeters,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0);
  private PIDController elevatorPID = new PIDController(15, 0, 0);
  private ElevatorState currentState = ElevatorState.Home;

  public ElevatorIOSim() {
    elevatorPID.setTolerance(0.01);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    switch (currentState) {
      case Home:
        seekPosition(0);
        break;
      case L2:
        seekPosition(0.2286);
        break;
      case L3:
        seekPosition(0.6096);
        break;
      case L4:
        seekPosition(1.3);
        break;
    }

    setVoltage(elevatorPID.calculate(m_elevatorSim.getPositionMeters()));

    m_elevatorSim.update(0.02);
    inputs.positionMeters = m_elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
    inputs.currentAmps = new double[] {m_elevatorSim.getCurrentDrawAmps()};
    inputs.state = currentState;
  }

  @Override
  public void setVoltage(double volts) {
    m_elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void seekPosition(double position) {
    elevatorPID.setSetpoint(position);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setElevatorState(ElevatorState state) {
    currentState = state;
  }
}
