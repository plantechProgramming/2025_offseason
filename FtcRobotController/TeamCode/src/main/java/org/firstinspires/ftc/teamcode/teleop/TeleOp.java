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
        Elevator lift = new Elevator(EA, EH, intake_center_angle, telemetry);

        boolean is_up = false;

        double forward; //-1 to 1
        double turn;
        double drift;
        double botHeading;
//        boolean liftFlagDown = false;
        EH.setDirection(DcMotorSimple.Direction.REVERSE);
        EH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeIsActive() ) {
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            drift = gamepad1.left_stick_x;
            botHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            ElapsedTime elapsedTime = new ElapsedTime();

            if(gamepad1.right_trigger > 0.0){
                driveTrain.drive(forward * 0.3, drift * 0.4, turn, botHeading);
            }else{
                driveTrain.drive(forward, drift, turn, botHeading);
            }

            if (gamepad1.a){roni2_intake.setPosition(1); }
            else if (gamepad1.b) {roni2_intake.setPosition(0);}


//

            lift.Intake_angle(gamepad1.dpad_left,gamepad1.dpad_right);

//            intake_center_angle.setPosition(0.57);

//            if (gamepad1.right_trigger>0){lift.set_wanted_height(2900); telemetry.addData("wanted", lift.wanted);}
//            else if (gamepad1.left_trigger>0) {lift.set_wanted_height(0);}
            lift.heightByPress(gamepad1.right_trigger,gamepad1.left_trigger);
//            lift.Change_Height();
            lift.Change_Angle(gamepad1.right_bumper,gamepad1.left_bumper);
//            if (gamepad1.left_bumper){lift.setAngleWanted(1285);}
//            else if (gamepad1.right_bumper) {lift.setAngleWanted(0);}
//            lift.Change_Angle_Pos();



//            telemetry.addData("EH",EH.getCurrentPosition());




            if(gamepad1.back){Imu.resetYaw();}

//            telemetry.addData("IMU: ", botHeading);
//            telemetry.addData("center x: ", DriveFrontRight.getCurrentPosition());
//            telemetry.addData("y: ", DriveBackLeft.getCurrentPosition());

            telemetry.addData("ea",EA.getCurrentPosition());
            telemetry.update();

        }

        EH.setPower(0);
    }
    @Override
    protected void end() {

    }


}
