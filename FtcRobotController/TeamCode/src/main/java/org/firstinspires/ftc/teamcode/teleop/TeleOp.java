package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;import static java.lang.Math.abs;
import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;

import org.firstinspires.ftc.teamcode.Elevator.Elevator;
import org.firstinspires.ftc.teamcode.OpMode;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    @Override
    protected void postInit() {
        Imu.resetYaw();
    }


    @Override
    public void run(){


        DriveTrain driveTrain = new DriveTrain(DriveBackRight, DriveBackLeft, DriveFrontRight, DriveFrontLeft, telemetry, Imu);
        Elevator lift = new Elevator(EA, EH, intake_center_angle, intake_left, intake_right, intake_AR, intAR, telemetry);

        boolean is_up = false;

        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean is_in = false;
        boolean is_down = false;


        while (opModeIsActive()) {
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if(gamepad1.right_trigger > 0.0){
                driveTrain.drive(forward * 0.3, drift * 0.4, turn, botHeading);
            }else{
                driveTrain.drive(forward, drift, turn, botHeading);
            }
//            while (gamepad1.right_trigger>0){lift.Change_Height(1);}
//            while (gamepad1.left_trigger>0) {lift.Change_Height(-1);}

            if (gamepad1.a){roni2_intake.setPosition(1); }
            else if (gamepad1.b) {roni2_intake.setPosition(0.75);}

            if (gamepad1.y){intake_center_angle.setPosition(1);}
            if (gamepad1.x){intake_center_angle.setPosition(0.57); }
//
            if (gamepad1.dpad_up){lift.Change_Height(3000);}
            telemetry.addData("EH",EH.getCurrentPosition());
            if (gamepad1.dpad_down) {lift.Change_Height(0);}
//
            if (gamepad1.dpad_left){lift.Change_Height(1);}
            if (gamepad1.dpad_right){lift.Change_Angle(1);}

//
//
//            if(gamepad1.right_bumper ){
//                intake_AR.setPosition(.8);
//                lift.extend(1,0.2);
//                intake_center_angle.setPosition(0.2);
//                sleep(500);
//
//                lift.move_intake_AG(0.15);
//                sleep(1000);
//                roni2_intake.setPosition(1);
//                sleep(1000);
//                intake_center_angle.setPosition(0.7);
//                lift.extend(-0.75, 1);
//
//
//
//            }  if (gamepad1.left_bumper ) {
//                lift.Move_Elevator(8000);
//                intake_AR.setPosition(0);
//                sleep(500);
//
//                intake_AR.setPosition(0.7);
//                sleep(500);
//                lift.Move_Elevator(-8000);
//                sleep(500);
//                intake_center_angle.setPosition(1);
//                sleep(100);
//
//            }


            if(gamepad1.back){Imu.resetYaw();}

            telemetry.addData("IMU: ", botHeading);
            telemetry.addData("center x: ", DriveFrontRight.getCurrentPosition());
            telemetry.addData("y: ", DriveBackLeft.getCurrentPosition());
            telemetry.update();

        }

    }
    @Override
    protected void end() {

    }


}
