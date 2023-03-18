
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// test change

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    // Xbox controller port
    // Add operator controller
    // Make sure to ask electrical for this necessary information

    // Motor controllers:
    private WPI_TalonSRX leftFrontLeadTalon;
    private WPI_TalonSRX leftFrontSupportTalon;
    private WPI_TalonSRX leftBackLeadTalon;
    private WPI_TalonSRX leftBackSupportTalon;
    private WPI_TalonSRX rightFrontLeadTalon;
    private WPI_TalonSRX rightFrontSupportTalon;
    private WPI_TalonSRX rightBackLeadTalon;
    private WPI_TalonSRX rightBackSupportTalon;

    private MecanumDrive mecanumDriveObj;
    private MecanumDrivePoseEstimator poseEstimator;

    // Drive Motor Encoders:
    private RelativeEncoder leftFrontEncoderObj;
    private RelativeEncoder leftBackEncoderObj;
    private RelativeEncoder rightFrontEncoderObj;
    private RelativeEncoder rightBackEncoderObj;

    // private JoystickSubsystem joystickSubsystemObj;

    // Pigeon IMU also used for gyro:
    private WPI_PigeonIMU pigeon;

    // TODO: Add odomtery class for tracking robot pose

    // Creates a new Drivetrain
    public Drivetrain(WPI_PigeonIMU pigeon) {
        // initialize stuff here

        // Spark max
        leftFrontLeadTalon = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_FRONT_TALON);
        leftFrontSupportTalon = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_FRONT_TALON);
        rightFrontLeadTalon = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_FRONT_TALON);
        rightFrontSupportTalon = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_FRONT_TALON);
        leftBackLeadTalon = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_BACK_TALON);
        leftBackSupportTalon = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_BACK_TALON);
        rightBackLeadTalon = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_BACK_TALON);
        rightBackSupportTalon = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_BACK_TALON);
        

        mecanumDriveObj = new MecanumDrive(leftFrontTalon, leftBackTalon, rightFrontTalon, rightBackTalon);

        // Encoder for the spark max
        // TODO: zero the wheel encoders
        // leftFrontEncoderObj = leftFrontTalon.get; // TODO: Correct countsPerRev
        // leftBackEncoderObj = leftBackTalon.getEncoder();
        // rightFrontEncoderObj = rightFrontTalon.getEncoder();
        // rightBackEncoderObj = rightBackTalon.getEncoder();
        this.pigeon = pigeon;

        leftFrontTalon.setInverted(true);
        rightFrontTalon.setInverted(true);

        // leftFrontTalon.setIdleMode(IdleMode.kBrake);
        // rightFrontTalon.setIdleMode(IdleMode.kBrake);
        // leftBackTalon.setIdleMode(IdleMode.kBrake);
        // rightBackTalon.setIdleMode(IdleMode.kBrake);

        // Pose/Orientation
        // poseEstimator = null;

        // need to limit current for mecanum to keep battery from dropping so low we
        // can't drive
        // leftFrontTalon.setSmartCurrentLimit(Constants.DRIVETRAIN_MOTOR_CURRENT_LIMIT_AMPS);
        // leftBackTalon.setSmartCurrentLimit(Constants.DRIVETRAIN_MOTOR_CURRENT_LIMIT_AMPS);
        // rightFrontTalon.setSmartCurrentLimit(Constants.DRIVETRAIN_MOTOR_CURRENT_LIMIT_AMPS);
        // rightBackTalon.setSmartCurrentLimit(Constants.DRIVETRAIN_MOTOR_CURRENT_LIMIT_AMPS);
        mecanumDriveObj.setMaxOutput(0.75);
        /*
         * maxOutput must be less than 1 to avoid overloading the battery and
         * not being able to drive.
         */

        // mecanumDriveObj.setSafetyEnabled(false);
    }

    public void setRightBackMotorInversion(boolean isInverted) {
        rightBackTalon.setInverted(isInverted);
    }

    public void setLeftFrontMotorInversion(boolean isInverted) {
        leftFrontTalon.setInverted(isInverted);
    }

    public void mecanumDriveCartesian(double xSpeed, double ySpeed, double zRotation) {
        // Creates a SlewRateLimiter that limits the rate of change of the signal
        // SlewRateLimiter filterFwdBack = new
        // SlewRateLimiter(Constants.FORWARD_BACK_SLEW_RATE);
        // SlewRateLimiter filterLeftRight = new
        // SlewRateLimiter(Constants.LEFT_RIGHT_SLEW_RATE);
        mecanumDriveObj.driveCartesian(xSpeed, ySpeed, zRotation);
        // mecanumDriveObj.driveCartesian(filterLeftRight.calculate(xSpeed),
        // filterFwdBack.calculate(ySpeed), zRotation);
    }

    public double getPitch() {
        double ypr[] = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[1];
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // scales the max speed of the drivetrain. default = 1
    // use <1 to slow down the drivetrain all the time
    public void setMaxOutput(double maxOutput) {
        mecanumDriveObj.setMaxOutput(maxOutput);
    }

    public double getAvgEncoderRotations() {
        double rotationSum = leftFrontEncoderObj.getPosition() + leftBackEncoderObj.getPosition()
                + rightFrontEncoderObj.getPosition() + rightBackEncoderObj.getPosition();
        return rotationSum / 4;
    }

    // returns distance in inches
    public double getAvgEncoderDistance() {
        return getAvgEncoderRotations() * Constants.DRIVETRAIN_WHEEL_DIAMETER * Math.PI * 1.575;
        // added 0.634 offset because robot is consistently off when commanding a
        // distance
    }

    public double getEncoderDistance() {
        return rightFrontEncoderObj.getPosition() * Constants.DRIVETRAIN_WHEEL_DIAMETER * Math.PI * 1.575;
        // added 0.634 offset because robot is consistently off when commanding a
        // distance
    }

    public void resetEncoders() {
        leftFrontEncoderObj.setPosition(0);
        rightFrontEncoderObj.setPosition(0);
        leftBackEncoderObj.setPosition(0);
        rightBackEncoderObj.setPosition(0);
    }

    /**
     * Returns the pose of the robot (e.g., x and y position of the robot on the
     * field and the robot's
     * rotation). The origin of the field to the lower left corner (i.e., the corner
     * of the field to
     * the driver's right). Zero degrees is away from the driver and increases in
     * the CCW direction.
     *
     * @return the pose of the robot
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();

    }
}
