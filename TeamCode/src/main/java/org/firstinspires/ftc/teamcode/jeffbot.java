package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "StarterBot", group = "StarterBot")
public class jeffbot extends OpMode {
    private Servo spinnyServo1 = null;
    private Servo spinnyServo2 = null;
    private DcMotor topMotor = null;
    private DcMotor leftjeff = null;
    private DcMotor rightjeff = null;
    double ljeffPower;
    double rjeffPower;

    @Override
    public void init() {

        spinnyServo1.setDirection(Servo.Direction.FORWARD);

        spinnyServo2.setDirection(Servo.Direction.FORWARD);

        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftjeff.setDirection(DcMotor.Direction.FORWARD);

        rightjeff.setDirection(DcMotor.Direction.FORWARD);

        leftjeff.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightjeff.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // gives the data that intization is dun
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

   @Override
   public void loop() {
    arcadeDrive(-gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x);
    telemetry.addData("Motors","left(%.2f), right(%.2f)", ljeffPower,rjeffPower);
   }
   void arcadeDrive(double forward, double rotate, double strafe) {
        ljeffPower = forward + rotate + strafe;
        rjeffPower = forward + rotate - strafe;
        leftjeff.setPower(ljeffPower);
        rightjeff.setPower(rjeffPower);
   }
}
