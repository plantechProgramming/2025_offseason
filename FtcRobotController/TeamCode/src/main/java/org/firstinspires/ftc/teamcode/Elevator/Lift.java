import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PID;

//public class Lift {
//    private DcMotorEx lift;
//    CRServo intake_center_angle;
//    public Lift(HardwareMap hardwareMap) {
//        lift = hardwareMap.get(DcMotorEx.class, "EH");
//        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        lift.setDirection(DcMotorEx.Direction.REVERSE);
//        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        intake_center_angle = hardwareMap.get(CRServo.class,"intA");
//    }
//
//    public class LiftUpSample implements Action {
//        private boolean initialized = false;
//        double power;
//        double thresh = 40;
//        PID pid_EH = new PID(0.15,0.05,0.1,0,0);
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
////                    if (!initialized) {
////                         lift.setPower(0.8);
////                        initialized = true;
////                    }
//            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            pid_EH.setWanted(2800);
//            if (Math.abs(2800 - lift.getCurrentPosition()) < thresh){
//                power = 0;
//                lift.setPower(power);
//                return false;
//
//            }
//            else{
//                power = pid_EH.update(lift.getCurrentPosition());
//                packet.put("liftPos", lift.getCurrentPosition());
//                lift.setPower(power);
//                return true;
//            }
//        }
//    }
//
//    public Action liftUpSample() {
//        return new LiftUpSample();
//    }
//
//}