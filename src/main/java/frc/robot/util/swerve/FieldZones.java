package frc.robot.util.swerve;

public enum FieldZones {
  Alliance,
  Neutral,
  Enemy,
  Launch;

  @Override
  public String toString() {
    switch (this) {
      case Alliance:
        return "Alliance Zone";
      case Neutral:
        return "Neutral Zone";
      case Enemy:
        return "Enemy Zone";
      case Launch:
        return "Launch Zone";
      default:
        return super.toString();
    }
  }
}