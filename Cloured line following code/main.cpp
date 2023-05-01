#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable
Pi2c arduino(4);

int x;
int y;
int z;

int blackpixels;
int bluepixels;
int redpixels;
int greenpixels;
int yellowpixels;

float distanceone = 160;
float distancetwo = 96;
float distancethree = 32;
float distancefour = -32;
float distancefive = -96;
float distancesix = -160;

float mainerror;
float errorsum;
float errordifference;
float u;
float servoAngle;
int setpoint = 0;
float previouserror;
float leftmotorspeed;
float rightmotorspeed;
int baseAngle = 92;
int basespeed = 250;
float K = 0.5;
float Kp = 0.5;
float Ki = 0.1;
float Kd = 0.25;

float one;
float two;
float three;
float four;
float five;
float six;
float total;
float weightedmean;

Mat image;
Mat image_HSV;
Mat image_PINK;
Mat image_RED;
Mat image_BLUE;
Mat image_GREEN;
Mat image_YELLOW;
Mat image_BLACK;

void redline();
void blueline();
void greenline();
void yellowline();
void blackline();
void linefollowing();
void pidcalculation();
void transmission();

void setup()
{
    setupCamera(320, 240);
}

int main( int argc, char** argv )
{
    setup();
    cv::namedWindow("Photo");   // Create a GUI window called photo
    while(1)    // Main loop to perform image processing
    {
        Mat frame;
        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        cv::imwrite("photo.jpg", frame);
        image = readImage("/home/pi/Desktop/OpenCV-Template/photo.jpg");

        cv::imshow("Photo", frame); //Display the image in the window

        cvtColor(frame, image_HSV, COLOR_BGR2HSV); // Convert the image to HSV

        blackline();

        x = 250;
        y = 250;

        transmission();

        if (u != 0) {
        x = leftmotorspeed;
        y = rightmotorspeed;
        z = servoAngle;
        transmission();
        } else {
        z = servoAngle;
        transmission();
        }

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)


        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
	}

	closeCV();  // Disable the camera and close any windows

	return 0;
}

void linefollowing()
{
Mat croppedImage;
croppedImage = image(Rect(0,0,240,1));
int pixelone = image.at<int>(0,0);
int pixeltwo = image.at<int>(0,64);
int pixelthree = image.at<int>(0,128);
int pixelfour = image.at<int>(0,192);
int pixelfive = image.at<int>(0,256);
int pixelsix = image.at<int>(0,320);
one = pixelone * distanceone;
two = pixeltwo * distancetwo;
three = pixelthree * distancethree;
four = pixelfour * distancefour;
five = pixelfive * distancefive;
six = pixelsix * distancesix;
total = pixelone + pixeltwo + pixelthree + pixelfour + pixelfive + pixelsix;
weightedmean = (one + two + three + four + five + six)/total;
printf("%c",weightedmean);
pidcalculation();
}

void blackline()
{
    inRange(image_HSV, Scalar(35, 68, 35), Scalar(179, 255, 65), image_BLACK);
    cv::namedWindow("Blackline");
    cv::imshow("Blackline", image_BLACK);
    blackpixels = countNonZero(image_BLACK);
    std::cout << "Black pixels = " << blackpixels << std::endl;
    if (blackpixels <= 1000)
    {
    redline();
    }
    else{
    image_BLACK = image;
    linefollowing();
    }
}

void redline()
{
    inRange(image_HSV, Scalar(105, 114, 113), Scalar(179, 255, 255), image_RED);
    cv::namedWindow("Redline");
    cv::imshow("Redline", image_RED);
    redpixels = countNonZero(image_RED);
    std::cout << "Red pixels = " << redpixels << std::endl;
    if (redpixels <= 1000)
    {
    blueline();
    }
    else{
    image_RED = image;
    linefollowing();
    }
}

void blueline()
{
    inRange(image_HSV, Scalar(35, 68, 57), Scalar(130, 255, 108), image_BLUE);
    cv::namedWindow("Blueline");
    cv::imshow("Blueline", image_BLUE);
    bluepixels = countNonZero(image_BLUE);
    std::cout << "Blue pixels = " << bluepixels << std::endl;
    if (bluepixels <= 1000)
    {
    greenline();
    }
    else{
    image_BLUE = image;
    linefollowing();
    }
}

void greenline()
{
    inRange(image_HSV, Scalar(35, 68, 97), Scalar(88, 255, 255), image_GREEN);
    cv::namedWindow("Greenline");
    cv::imshow("Greenline", image_GREEN);
    greenpixels = countNonZero(image_GREEN);
    std::cout << "Blue pixels = " << greenpixels << std::endl;
    if (greenpixels <= 1000)
    {
    yellowline();
    }
    else{
    image_GREEN = image;
    linefollowing();
    }
}

void yellowline()
{
    inRange(image_HSV, Scalar(0, 210, 141), Scalar(48, 255, 255), image_YELLOW);
    cv::namedWindow("Yellowline");
    cv::imshow("Yellowline", image_YELLOW);
    yellowpixels = countNonZero(image_YELLOW);
    std::cout << "Yellow pixels = " << yellowpixels << std::endl;
    if (yellowpixels > 1000)
    {
    image_YELLOW = image;
    linefollowing();
    }
    else{
    return;
    }
}

void pidcalculation()
{
    mainerror = setpoint - weightedmean;
    errorsum = errorsum + mainerror;
  if (mainerror != previouserror){
  errordifference = mainerror - previouserror;
  previouserror = mainerror;
  u = (Kp * mainerror) + (Ki * errorsum) + (Kd * errordifference);
  servoAngle = baseAngle + u;
  leftmotorspeed = basespeed + (K * u);
  rightmotorspeed = basespeed - (K * u);
  return;
  }
}

void transmission()
{
        arduino.i2cWriteArduinoInt(x);
        arduino.i2cWriteArduinoInt(y);
        arduino.i2cWriteArduinoInt(z);
}
