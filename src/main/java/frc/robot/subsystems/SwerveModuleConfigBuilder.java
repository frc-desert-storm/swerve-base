package frc.robot.subsystems;

public class SwerveModuleConfigBuilder {

    private Integer driveCanId;
    private Integer steerCanId;
    private Integer encoderCanId;

    public SwerveModuleConfigBuilder driveMotor(int id) {
        this.driveCanId = id;
        return this;
    }

    public SwerveModuleConfigBuilder steerMotor(int id) {
        this.steerCanId = id;
        return this;
    }

    public SwerveModuleConfigBuilder encoder(int id) {
        this.encoderCanId = id;
        return this;
    }

    public SwerveModuleConfig build() {
        if (this.driveCanId == null) throw new IllegalStateException("driveCanId not set");
        if (this.steerCanId == null) throw new IllegalStateException("steerCanId not set");
        if (this.encoderCanId == null) throw new IllegalStateException("encoderCanId not set");

        return new SwerveModuleConfig(driveCanId, steerCanId, encoderCanId);
    }
}
