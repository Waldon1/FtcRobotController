package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "MecanumPointofViewDrive2022")
public class MecanumPointofViewDrive2022 extends LinearOpMode {

    private DcMotor rightfront;
    private DcMotor rightback;
    private DcMotor leftfront;
    private DcMotor leftback;
    private Servo GripperServo;
    private CRServo lift;

    boolean Last_Value_of_Gripper_Position;
    boolean Last_Value_of_Swivel;
    float forward;
    double Gripper_Position2;
    double Swivel_Position;
    float strafe;
    float turn;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double denominator;

        rightfront = hardwareMap.get(DcMotor.class, "right front");
        rightback = hardwareMap.get(DcMotor.class, "right back");
        leftfront = hardwareMap.get(DcMotor.class, "left front");
        leftback = hardwareMap.get(DcMotor.class, "left back");
        Servo armSwivl = hardwareMap.get(Servo.class, "armSwivl");
        GripperServo = hardwareMap.get(Servo.class, "GripperServo");
        lift = hardwareMap.get(CRServo.class, "lift");

        // Put initialization blocks here.
        // this is for the direction of the wheels
        rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightback.setDirection(DcMotorSimple.Direction.REVERSE);
        leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftback.setDirection(DcMotorSimple.Direction.FORWARD);
        // this is starting the lift encoder
        // this sets 0 power behavior to stopping the motor
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // this is the linear slide and the gripper varables
        armSwivl.setPosition(0);
        Swivel_Position = 0.47;
        Last_Value_of_Swivel = false;
        armSwivl.setPosition(Swivel_Position);
        Gripper_Position2 = 0.5;
        Last_Value_of_Gripper_Position = false;
        GripperServo.setPosition(Gripper_Position2);
        // this is waiting for driver to push start
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // this for setting the drive speed based of inputs
                forward = gamepad1.left_stick_y;
                strafe = -gamepad1.left_stick_x;
                turn = -gamepad1.right_stick_x;
                // this is for setting speed to the controller inputs
                driveSpeed();
                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(forward) + Math.abs(strafe + Math.abs(turn))));
                // makes sure inputs are set than drive speed
                // this is setting the motor power to inputs
                leftfront.setPower((forward + strafe + turn) / denominator);
                leftback.setPower(((forward - strafe) + turn) / denominator);
                rightfront.setPower(((forward - strafe) - turn) / denominator);
                rightback.setPower(((forward + strafe) - turn) / denominator);
                // this is for manual linear slide movment
                Gripper_Position();
                manaulSetLIftPos();
                Drive_Forward_Dpad();
                Drive_Backward_Dpad();
                Strafe_Left_Dpad();
                Strafe_Right_Dpad();
                Drive_Forward_Letter();
                Drive_Backward_Letter();
                Strafe_Right_Letter();
                Strafe_Left_Letter();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void callStop() {
        // this means that if said to stop, it stops
        if (isStopRequested()) {
            terminateOpModeNow();
        }
    }

    /**
     * Describe this function...
     */
    private void driveSpeed() {
        // Setting the drive speed.
        if (gamepad1.left_bumper) {
            forward = (float) (forward * 0.75);
            strafe = (float) (strafe * 0.75);
            turn = (float) (turn * 0.75);
        } else if (gamepad1.right_bumper) {
            forward = (float) (forward * 0.35);
            strafe = (float) (strafe * 0.35);
            turn = (float) (turn * 0.35);
        } else {
            forward = (float) (forward * 0.5);
            strafe = (float) (strafe * 0.5);
            turn = (float) (turn * 0.5);
        }
    }

    /**
     * Describe this function...
     */
    private void Gripper_Position() {
        if (!Last_Value_of_Gripper_Position && gamepad2.a) {
            if (Gripper_Position2 == 0.4) {
                Gripper_Position2 = 0.75;
            } else {
                Gripper_Position2 = 0.4;
            }
        }
        Last_Value_of_Gripper_Position = gamepad2.a;
        GripperServo.setPosition(Gripper_Position2);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void manaulSetLIftPos() {
        // Sets drive speed.
        if (gamepad2.dpad_down) {
            if (gamepad2.right_bumper) {
                lift.setPower(-0.25);
            } else {
                lift.setPower(-0.5);
            }
        } else if (gamepad2.dpad_up) {
            if (gamepad2.right_bumper) {
                lift.setPower(0.25);
            } else {
                lift.setPower(0.6);
            }
        } else {
            lift.setPower(0.08);
        }
    }

    /**
     * Describe this function...
     */
    private void Drive_Backward_Dpad() {
        while (gamepad1.dpad_right) {
            if (gamepad1.dpad_right) {
                leftfront.setPower(0.4);
                leftback.setPower(0.4);
                rightfront.setPower(0.4);
                rightback.setPower(0.4);
            } else {
                leftfront.setPower(0);
                leftback.setPower(0);
                rightfront.setPower(0);
                rightback.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Drive_Forward_Dpad() {
        while (gamepad1.dpad_left) {
            if (gamepad1.dpad_left) {
                leftfront.setPower(-0.4);
                leftback.setPower(-0.4);
                rightfront.setPower(-0.4);
                rightback.setPower(-0.4);
            } else {
                leftfront.setPower(0);
                leftback.setPower(0);
                rightfront.setPower(0);
                rightback.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Strafe_Right_Dpad() {
        while (gamepad1.dpad_up) {
            if (gamepad1.dpad_up) {
                leftfront.setPower(-0.4);
                leftback.setPower(0.4);
                rightfront.setPower(0.4);
                rightback.setPower(-0.4);
            } else {
                leftfront.setPower(0);
                leftback.setPower(0);
                rightfront.setPower(0);
                rightback.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Strafe_Left_Dpad() {
        while (gamepad1.dpad_down) {
            if (gamepad1.dpad_down) {
                leftfront.setPower(0.4);
                leftback.setPower(-0.4);
                rightfront.setPower(-0.4);
                rightback.setPower(0.4);
            } else {
                leftfront.setPower(0);
                leftback.setPower(0);
                rightfront.setPower(0);
                rightback.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Drive_Backward_Letter() {
        while (gamepad1.x) {
            if (gamepad1.x) {
                leftfront.setPower(0.4);
                leftback.setPower(0.4);
                rightfront.setPower(0.4);
                rightback.setPower(0.4);
            } else {
                leftfront.setPower(0);
                leftback.setPower(0);
                rightfront.setPower(0);
                rightback.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Drive_Forward_Letter() {
        while (gamepad1.b) {
            if (gamepad1.b) {
                leftfront.setPower(-0.4);
                leftback.setPower(-0.4);
                rightfront.setPower(-0.4);
                rightback.setPower(-0.4);
            } else {
                leftfront.setPower(0);
                leftback.setPower(0);
                rightfront.setPower(0);
                rightback.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Strafe_Right_Letter() {
        while (gamepad1.a) {
            if (gamepad1.a) {
                leftfront.setPower(-0.4);
                leftback.setPower(0.4);
                rightfront.setPower(0.4);
                rightback.setPower(-0.4);
            } else {
                leftfront.setPower(0);
                leftback.setPower(0);
                rightfront.setPower(0);
                rightback.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Strafe_Left_Letter() {
        while (gamepad1.y) {
            if (gamepad1.y) {
                leftfront.setPower(0.4);
                leftback.setPower(-0.4);
                rightfront.setPower(-0.4);
                rightback.setPower(0.4);
            } else {
                leftfront.setPower(0);
                leftback.setPower(0);
                rightfront.setPower(0);
                rightback.setPower(0);
            }
        }
    }
}
