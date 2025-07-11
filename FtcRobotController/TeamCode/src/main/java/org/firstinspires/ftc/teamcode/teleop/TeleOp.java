package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        Elevator lift = new Elevator(EA, EH, intake_center_angle,roni2_intake, telemetry);

        boolean is_up = false;

        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
        boolean slow = false;

        EH.setDirection(DcMotorSimple.Direction.REVERSE);
        EH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeIsActive() ) {
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            ElapsedTime elapsedTime = new ElapsedTime();
            driveTrain.drive(forward, drift, turn, botHeading, 1);


            if(gamepad1.x && !slow){
                driveTrain.drive(forward, drift, turn, botHeading, 0.5);
                slow = true;
            }else if(gamepad1.x && slow) {
                driveTrain.drive(forward, drift, turn, botHeading, 1);
                slow = false;
                telemetry.addData("y: ", DriveBackLeft.getCurrentPosition());
                telemetry.addData("x:", DriveFrontRight.getCurrentPosition());
            }


            if (gamepad1.a){roni2_intake.setPosition(0.7); }
            else if (gamepad1.b) {roni2_intake.setPosition(0);}

            if(gamepad1.y){lift.preload();}


            lift.Intake_Angle(gamepad1.dpad_down,gamepad1.dpad_up);
//            if (gamepad1.dpad_left){intake_center_angle.setPosition(1);}
//            if(gamepad2.dpad_right){intake_center_angle.setPosition(0);}

//            intake_center_angle.setPosition(0.57);

//            if (gamepad1.right_trigger>0){lift.set_wanted_height(2900); telemetry.addData("wanted", lift.wanted);}
//            else if (gamepad1.left_trigger>0) {lift.set_wanted_height(0);}
            lift.heightByPress(gamepad1.right_trigger,gamepad1.left_trigger);

            lift.Change_Angle(gamepad1.right_bumper,gamepad1.left_bumper);

            if(gamepad1.back){Imu.resetYaw();}

            telemetry.addData("ea",EA.getCurrentPosition());
            telemetry.addData("eH",EH.getCurrentPosition());

            telemetry.update();

        }
        EH.setPower(0);
    }

    @Override
    protected void end() {

    }


}
