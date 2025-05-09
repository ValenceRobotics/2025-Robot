package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ElevatorMath;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {

  public static final DCMotor m_elevatorMotors = DCMotor.getNEO(2);
  public static final double kElevatorGearing = 6;
  public static final double kCarriageMassKg = Units.lbsToKilograms(25);
  public static final double kElevatorDrumRadiusMeters = Units.inchesToMeters(0.829 / 2);
  public static final double kMinElevatorHeightMeters = 0;
  public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(59.035);

  public static final int motorId1 = 11;
  public static final int motorId2 = 12;

  public static final int currentLimit = 40;
  public static final double kPSim = 0.9;
  public static final double kDSim = 0;
  public static final double kGSim = 0.39;
  public static final double kVSim =
      9.21 / ElevatorMath.convertDistanceToRotations(Meters.of(1)).in(Rotations);

  public static LoggedTunableNumber kPReal = new LoggedTunableNumber("Elevator/kP", 0.065);
  public static LoggedTunableNumber kDReal = new LoggedTunableNumber("Elevator/kD", 0.001);
  public static LoggedTunableNumber kGReal = new LoggedTunableNumber("Elevator/kG", 0.4);
  public static final double kVReal =
      9.21 / ElevatorMath.convertDistanceToRotations(Meters.of(1)).in(Rotations);

  public static final double maxVelocity = 30 * 60;
  public static final double maxAcceleration = 1 * 36 * 60;

  public static final double allowedError =
      ElevatorMath.convertDistanceToRotations(Meters.of(0.01)).in(Rotations);
}
