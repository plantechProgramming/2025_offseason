package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Elevator.Elevator;
import org.firstinspires.ftc.teamcode.OpMode;

@Autonomous
public class test extends OpMode {
    DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
    Elevator lift = new Elevator(armL, armR, intake_center_angle, intake_left, intake_right, intake_AR, intAR, telemetry);

    @Override
    protected void run() {
        diagonal(driveTrain,0.2, 0.5);
    }

    @Override
    protected void end() {

    }
}
