package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public final class TwoDeadWheelInputsMessage {
    public long timestamp;
    public PositionVelocityPair DriveBackLeft;
    public PositionVelocityPair DriveFrontRight;
    public double yaw;
    public double pitch;
    public double roll;
    public double xRotationRate;
    public double yRotationRate;
    public double zRotationRate;

    public TwoDeadWheelInputsMessage(PositionVelocityPair DriveBackLeft, PositionVelocityPair DriveFrontRight, YawPitchRollAngles angles, AngularVelocity angularVelocity) {
        this.timestamp = System.nanoTime();
        this.DriveBackLeft = DriveBackLeft;
        this.DriveFrontRight = DriveFrontRight;
        {
            this.yaw = angles.getYaw(AngleUnit.RADIANS);
            this.pitch = angles.getPitch(AngleUnit.RADIANS);
            this.roll = angles.getRoll(AngleUnit.RADIANS);
        }
        {
            this.xRotationRate = angularVelocity.xRotationRate;
            this.yRotationRate = angularVelocity.yRotationRate;
            this.zRotationRate = angularVelocity.zRotationRate;
        }
    }
}
