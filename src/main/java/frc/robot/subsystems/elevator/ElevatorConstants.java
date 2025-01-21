package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

  public static final DCMotor m_elevatorMotors = DCMotor.getNEO(2);
  public static final double kElevatorGearing = 6;
  public static final double kCarriageMassKg = Units.lbsToKilograms(16.142);
  public static final double kElevatorDrumRadiusMeters = Units.inchesToMeters(0.829 / 2);
  public static final double kMinElevatorHeightMeters = 0;
  public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(59.035);
}
