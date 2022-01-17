package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.max;
import static java.lang.Math.min;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import java.util.List;

/*
 * TeleOp Version 1 For Ball Drive
 */
@TeleOp(name = "TeleOp BD")
public class TeleOpBallDrive extends OpMode
{
    //Initializing all necessary variables
    float y;
    float x;

    public DcMotor leftFront, leftRear, rightRear, rightFront;
    public DcMotor arm;
    //public DcMotor backCarousel, frontCarousel, arm;
    //public CRServo intake;
    public CRServo slider;

    public double armPosScale = 64, armPowScale = 0.0025;
    public double armPosCurrent=0, armPosDes=0, armPosError=0, armPow=0;
    public double integrater = 0.001, intpower = 0.00075, multiplier = 1, speedK = 1;
    

    //This is where all the variables used in the main program are initialized
    @Override
    public void init()
    {
        //Assigning gamepad values to hardware parts
        y = gamepad1.left_stick_y;
        x = gamepad1.right_stick_x;
        
        //Defining all the hardware parts
        //Quoted name is entered into robot config only

        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
       // backCarousel = hardwareMap.get(DcMotor.class, "flyback");
      //  frontCarousel = hardwareMap.get(DcMotor.class, "flyfront");
        arm = hardwareMap.get(DcMotor.class, "arm");
      //  intake = hardwareMap.get(CRServo.class, "intake");
        slider = hardwareMap.get(CRServo.class, "slider");


        //Setting the direction of each motor
        //Opposite side motors are reversed to move in the same direction
        rightRear.setDirection(DcMotor.Direction.REVERSE);                                  //alternating between forward and reverse depending on motor placement
        leftRear.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        //Initializing wheel variables at a constant
        float powerXWheels = 0;
        float powerYWheels = 0;

        //Handle regular movement
        powerYWheels += gamepad1.left_stick_y;

        //Handle sliding movement
        powerXWheels += gamepad1.right_stick_x;

        // Handle turning movement
        double maxX = (double) powerXWheels;
        double maxY = (double) powerYWheels;

        //Give X motors at most 75% power
        leftRear.setPower((Math.abs(maxX)*maxX) * -0.75);
        rightFront.setPower((Math.abs(maxX)*maxX) * -0.75);

        // Give Y motors at most 75% power
        rightRear.setPower((Math.abs(maxY)*maxY) * 0.75);
        leftFront.setPower((Math.abs(maxY)*maxY) * 0.75);

        //Turning clockwise with a 50% power scale on the motors
        if (gamepad1.right_trigger > 0)
        {
            rightFront.setPower(-0.5 * gamepad1.right_trigger);
            leftFront.setPower(-0.5 * gamepad1.right_trigger);
            leftRear.setPower(0.5 * gamepad1.right_trigger);
            rightRear.setPower(0.5 * gamepad1.right_trigger);
        }

        //Turning anticlockwise with a 50% power scale on the motors
        if (gamepad1.left_trigger > 0)
        {

            rightFront.setPower(0.5 * gamepad1.left_trigger);
            leftFront.setPower(0.5 * gamepad1.left_trigger);
            leftRear.setPower(-0.5 * gamepad1.left_trigger);
            rightRear.setPower(-0.5 * gamepad1.left_trigger);
        }

        //Hold X motor position when no power supplied
        if (gamepad1.right_stick_x == 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
        {
            rightFront.setPower(0);
            leftRear.setPower(0);
        }

        //Hold Y motor position when no power supplied
        if (gamepad1.left_stick_y == 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
        {
            leftFront.setPower(0);
            rightRear.setPower(0);
        }

        slider.setPower(gamepad2.right_stick_y);
/*
        //Right and left triggers run the intake servo
        if (gamepad2.right_bumper == true) {
            intake.setPower(1);
        } else if(gamepad2.left_bumper == true) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        //Red button for red carousel, blue button for blue carousel
        if (gamepad2.x == true) {
            backCarousel.setPower(0.375);
            frontCarousel.setPower(0.375);
        } else if(gamepad2.b == true) {
            backCarousel.setPower(-0.375);
            frontCarousel.setPower(-0.375);
        } else {
            backCarousel.setPower(0);
            frontCarousel.setPower(0);
        }
*/

        //Get the encoder position for the arm
        armPosCurrent = arm.getCurrentPosition();
        //Pure Proportional Feedback
        armPosDes += speedK*armPosScale*gamepad2.left_stick_y;
        //input scale factor
        armPosError = armPosDes - armPosCurrent;
        //integrater += armPosError;                                           //unnecessary
        armPow = Math.min(Math.max(armPowScale*armPosError, -1.00), 1.00);
        //proportional gain
        if(armPow >= 1){ armPosDes = armPosCurrent+(1/armPowScale); }       //AntiWindup Code
        if(armPow <= -1) {armPosDes = armPosCurrent-(1/armPowScale); }      //AntiWindup Code
        //Accelerate arms set motor based on proximity to desired position
        arm.setPower(armPow);
    }

    //Exit loop after OpMode Stop is requested
    @Override
    public void stop()
    {
    }
}