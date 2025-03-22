package org.firstinspires.ftc.teamcode.Auto;

import androidx.activity.result.contract.ActivityResultContracts;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.OpMode;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="test :(", group="Robot")
public class test extends OpMode {
    ElapsedTime time = new ElapsedTime();
    @Override
    protected void run() {
        Imu.resetYaw();
        Telemetry telemetry = new MultipleTelemetry();
        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
        turn(driveTrain, 90);
        telemetry.update();

    }

    @Override
    protected void end() {

    }
}
