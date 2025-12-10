package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBot", group = "StarterBot")
public class jeffbot extends OpMode {
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    private CRServo spinnyServo1 = null;
    private CRServo spinnyServo2 = null;
    private DcMotorEx topMotor = null;
    private DcMotor leftjeff = null;
    private DcMotor rightjeff = null;
    double ljeffPower;
    double rjeffPower;

    ElapsedTime feederTimer = new ElapsedTime();
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;
    @Override
    public void init() {

        launchState = LaunchState.IDLE;

        spinnyServo1 = hardwareMap.get(CRServo.class, "jeff1");

        spinnyServo2 = hardwareMap.get(CRServo.class, "jeff2");

        topMotor = hardwareMap.get(DcMotorEx.class, "mjefft");

        leftjeff = hardwareMap.get(DcMotor.class, "mjeffl");

        rightjeff = hardwareMap.get(DcMotor.class, "mjeffr");

        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftjeff.setDirection(DcMotor.Direction.REVERSE);

        rightjeff.setDirection(DcMotor.Direction.FORWARD);

        leftjeff.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightjeff.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinnyServo1.setPower(STOP_SPEED);

        spinnyServo2.setPower(STOP_SPEED);

        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        spinnyServo1.setDirection(DcMotorSimple.Direction.REVERSE);
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
    arcadeDrive(-gamepad1.left_stick_y,gamepad1.right_stick_x);
    if (gamepad1.y) {
        topMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
    } else if (gamepad1.b) {
        topMotor.setVelocity(STOP_SPEED);
    } else if (gamepad1.x) {
        topMotor.setVelocity(6000);
    }
    launch(gamepad1.rightBumperWasPressed());
    telemetry.addData("State", launchState);
    telemetry.addData("Motors","left(%.2f), right(%.2f)", ljeffPower,rjeffPower);
    telemetry.addData("motorSpeed", topMotor.getVelocity());
   }
   void arcadeDrive(double forward, double rotate) {
        ljeffPower = forward + rotate;
        rjeffPower = forward - rotate;
        leftjeff.setPower(ljeffPower);
        rightjeff.setPower(rjeffPower);
   }
   void launch(boolean shotRequested) {
       switch (launchState) {
           case IDLE:
               if (shotRequested) {
                   launchState = LaunchState.SPIN_UP;
               }
               break;
           case SPIN_UP:
               topMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
               if (topMotor.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                   launchState = LaunchState.LAUNCH;
               }
               break;
           case LAUNCH:
               spinnyServo1.setPower(FULL_SPEED);
               spinnyServo2.setPower(FULL_SPEED);
               feederTimer.reset();
               launchState = LaunchState.LAUNCHING;
               break;
           case LAUNCHING:
               if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                   launchState = LaunchState.IDLE;
                   spinnyServo1.setPower(STOP_SPEED);
                   spinnyServo2.setPower(STOP_SPEED);
               }
               break;
       }
    }
}
