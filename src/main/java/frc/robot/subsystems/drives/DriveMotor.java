/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drives;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Add your docs here.
 */
public class DriveMotor extends TalonSRX implements SpeedController {

    public static final double MAX_SPEED = 4300.0;
    public static final int MAX_PEAK_AMPS = 80;
    public static final int MAX_PEAK_DURATION = 200;
    public static final int MAX_CONTINUOUS_AMPS = 39;

    private double maxSpeed = MAX_SPEED;

    public DriveMotor(int talonCANId) {
        super(talonCANId);
        configVoltageCompSaturation(12.0, 0);
        enableVoltageCompensation(true);
    }

    @Override
    public void pidWrite(double output) {

    }

    @Override
    public void set(double speed) {
        this.set(ControlMode.Velocity, speed * maxSpeed);
    }

    @Override
    public double get() {
        return this.getMotorOutputPercent();
    }

    @Override
    public void disable() {
        //Not sure what we want to do
    }

    @Override
    public void stopMotor() {
        this.set(ControlMode.PercentOutput, 0.0);
    }
}
