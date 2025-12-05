package frc.robot.subsystems;

public class SwerveModuleConfig {
    public final int driveCanId;
    public final int steerCanId;
    public final int encoderCanId;

    public SwerveModuleConfig(int driveCanId, int steerCanId, int encoderCanId) {
        this.driveCanId = driveCanId;
        this.steerCanId = steerCanId;
        this.encoderCanId = encoderCanId;
    }

    public static SwerveModuleConfigBuilder builder() {
        return new SwerveModuleConfigBuilder();
    }

}
