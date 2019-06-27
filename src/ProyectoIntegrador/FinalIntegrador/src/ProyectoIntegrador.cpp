/**
 * @function moments_demo.cpp
 * @brief Demo code to calculate moments
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"
#include "../../../MicroDron/MicroDronInterface.h"
#include <iostream>
#include <iomanip>

using namespace cv;
using namespace std;
///Gota de aceite mira
vector<Point> points;
bool newPoint = false;
Vec3b blanco = {0, 0, 0};
Vec3b color2 = {0, 0, 0};

///Detector de Piel
///Declaracion de Variables
double min_face_size = 20;
double max_face_size = 200;
bool calibrated2 = false;

void removeBackground(Mat input, Mat background);

void drawSkinColorSampler(Mat input);

void calibrate(Mat input);

void calibrateB(Mat input);

void movDron(Mat camara, int angulo, int movimiento);

void removeFaces(Mat input, Mat output);

Mat getForeground(Mat input);

Mat getForegroundMask(Mat input);

Mat getSkinMask(Mat input);

Mat background, drawing;

MicroDronInterface *microDron;
VideoCapture camera;

int anguloYaw = 0;
int hLowThreshold = 0;
int hLowThreshold2 = 0;
int hLowThreshold3 = 0;
int hHighThreshold = 0;
int sLowThreshold = 0;
int sHighThreshold = 0;
int vLowThreshold = 0;
int vHighThreshold = 0;
int hHighThreshold2 = 0;
int sLowThreshold2 = 0;
int sHighThreshold2 = 0;
int vLowThreshold2 = 0;
int vHighThreshold2 = 0;
int hHighThreshold3 = 0;
int sLowThreshold3 = 0;
int sHighThreshold3 = 0;
int vLowThreshold3 = 0;
int vHighThreshold3 = 0;
bool calibrated = false;
Rect skinColorSamplerRectangle1, skinColorSamplerRectangle2;
Rect skinColorSamplerRectangle3, skinColorSamplerRectangle4;

void calculateThresholds(Mat sample1, Mat sample2, Mat sample3, Mat sample4);

void performOpening(Mat binaryImage, int structuralElementShapde, Point structuralElementSize);

bool activado = false;
bool shouldCalibrate_, shouldCalibrateB_, shouldActivate_, shouldStop_;
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void drawSkinColorSampler(Mat input) {
    int frameWidth = input.size().width, frameHeight = input.size().height;
    int rectangleSize = 40;
    Scalar rectangleColor = Scalar(255, 0, 255);

    skinColorSamplerRectangle1 = Rect(frameWidth / 5, frameHeight / 2, rectangleSize, rectangleSize);
    skinColorSamplerRectangle2 = Rect(frameWidth / 5, frameHeight / 3, rectangleSize, rectangleSize);
    skinColorSamplerRectangle3 = Rect(frameWidth / 4.5, frameHeight / 2, rectangleSize, rectangleSize);
    skinColorSamplerRectangle4 = Rect(frameWidth / 4.5, frameHeight / 3, rectangleSize, rectangleSize);

    rectangle(
            input,
            skinColorSamplerRectangle1,
            rectangleColor
    );
    rectangle(
            input,
            skinColorSamplerRectangle2,
            rectangleColor
    );
    rectangle(
            input,
            skinColorSamplerRectangle3,
            rectangleColor
    );
    rectangle(
            input,
            skinColorSamplerRectangle4,
            rectangleColor
    );
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void calibrate(Mat input) {
    Mat hsvInput;
    cvtColor(input, hsvInput, CV_BGR2HSV);
    Mat sample1 = Mat(hsvInput, skinColorSamplerRectangle1);
    Mat sample2 = Mat(hsvInput, skinColorSamplerRectangle2);
    Mat sample3 = Mat(hsvInput, skinColorSamplerRectangle3);
    Mat sample4 = Mat(hsvInput, skinColorSamplerRectangle4);
    calculateThresholds(sample1, sample2, sample3, sample4);
    calibrated = true;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void calibrateB(Mat input) {
    cvtColor(input, background, CV_BGR2GRAY);
    calibrated2 = true;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void calculateThresholds(Mat sample1, Mat sample2, Mat sample3, Mat sample4) {
    int offsetLowThreshold = 80;
    int offsetHighThreshold = 30;

    Scalar hsvMeansSample1 = mean(sample1);
    Scalar hsvMeansSample2 = mean(sample2);
    Scalar hsvMeansSample3 = mean(sample3);
    Scalar hsvMeansSample4 = mean(sample4);

    hLowThreshold2 = min(hsvMeansSample1[0], hsvMeansSample2[0]);
    hLowThreshold3 = min(hsvMeansSample3[0], hsvMeansSample4[0]);
    hHighThreshold2 = max(hsvMeansSample1[0], hsvMeansSample2[0]);
    hHighThreshold3 = max(hsvMeansSample3[0], hsvMeansSample4[0]);
    sLowThreshold2 = min(hsvMeansSample1[1], hsvMeansSample2[1]);
    sLowThreshold3 = min(hsvMeansSample3[1], hsvMeansSample4[1]);
    sHighThreshold2 = max(hsvMeansSample1[1], hsvMeansSample2[1]);
    sHighThreshold3 = max(hsvMeansSample3[1], hsvMeansSample4[1]);
    vLowThreshold2 = min(hsvMeansSample1[2], hsvMeansSample2[2]);
    vLowThreshold3 = min(hsvMeansSample3[2], hsvMeansSample4[2]);
    vHighThreshold2 = max(hsvMeansSample1[2], hsvMeansSample2[2]);
    vHighThreshold3 = max(hsvMeansSample3[2], hsvMeansSample4[2]);
    hLowThreshold = min(hLowThreshold2, hLowThreshold3) - offsetLowThreshold;
    hHighThreshold = max(hHighThreshold2, hHighThreshold3) + offsetHighThreshold;
    sLowThreshold = min(sLowThreshold2, sLowThreshold3) - offsetLowThreshold;
    sHighThreshold = max(sHighThreshold2, sHighThreshold3) + offsetHighThreshold;
    vLowThreshold = min(vLowThreshold2, vLowThreshold3) - offsetLowThreshold;
    vHighThreshold = max(vHighThreshold2, vHighThreshold3) + offsetHighThreshold;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

Mat getSkinMask(Mat input) {
    Mat skinMask;

    if (!calibrated) {
        skinMask = Mat::zeros(input.size(), CV_8UC1);
        return skinMask;
    }

    Mat hsvInput;
    cvtColor(input, hsvInput, CV_BGR2HSV);

    inRange(
            hsvInput,
            Scalar(hLowThreshold, sLowThreshold, vLowThreshold),
            Scalar(hHighThreshold, sHighThreshold, vHighThreshold),
            skinMask);

    performOpening(skinMask, MORPH_ELLIPSE, {3, 3});
    dilate(skinMask, skinMask, Mat(), Point(-1, -1), 3);

    return skinMask;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void performOpening(Mat binaryImage, int kernelShape, Point kernelSize) {
    Mat structuringElement = getStructuringElement(kernelShape, kernelSize);
    morphologyEx(binaryImage, binaryImage, MORPH_OPEN, structuringElement);
}

///Detector de Piel
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

Mat getForeground(Mat input) {
    Mat foregroundMask = getForegroundMask(input);
    Mat foreground;
    input.copyTo(foreground, foregroundMask);

    return foreground;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

Mat getForegroundMask(Mat input) {
    Mat foregroundMask;

    if (!calibrated2) {
        foregroundMask = Mat::zeros(input.size(), CV_8UC1);
        return foregroundMask;
    }
    cvtColor(input, foregroundMask, CV_BGR2GRAY);
    removeBackground(foregroundMask, background);

    return foregroundMask;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void removeBackground(Mat input, Mat background) {
    int thresholdOffset = 10;
    for (int i = 0; i < input.rows; i++) {
        for (int j = 0; j < input.cols; j++) {
            uchar framePixel = input.at<uchar>(i, j);
            uchar bgPixel = background.at<uchar>(i, j);

            if (framePixel >= bgPixel - thresholdOffset && framePixel <= bgPixel + thresholdOffset)
                input.at<uchar>(i, j) = 0;
            else
                input.at<uchar>(i, j) = 255;
        }
    }
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

int drawLine(Mat img, Moments Mom, Point p, string iString) {
    double angle = 0.5 * atan2(2 * Mom.mu11, Mom.mu20 - Mom.mu02) + 3.1416 / 2;
    double angle2 = angle - 3.1415 / 2;
    int angle3 = angle * 180 / 3.1415;

    if (angle3 <= 180 && angle3 >= 91) {
        angle3 = angle3 - 180;
    }

    Point p1 = Point(p.x - (int) (sin(angle) * 110), p.y + (int) (cos(angle) * 110));
    Point p2 = Point(p.x + (int) (sin(angle) * 110), p.y - (int) (cos(angle) * 110));
    Point p3 = Point(p.x - (int) (sin(angle2) * 20), p.y + (int) (cos(angle2) * 20));
    Point p4 = Point(p.x + (int) (sin(angle2) * 20), p.y - (int) (cos(angle2) * 20));

    line(img, p1, p2, {39, 253, 69}, 2);
    line(img, p3, p4, {39, 253, 69}, 2);
    int font = FONT_HERSHEY_SIMPLEX;

    putText(img, iString, Point(0, 50), font, 1, {0, 0, 0}, 2);

    if (angle3 > 20) {
        //putText(img, "Left" , Point(100,100) , font, 1,{0,0,0},2);
        return 1;
    } else if (angle3 < -20) {
        //putText(img, "Right" , Point(100,100) , font, 1,{0,0,0},2);
        return 2;
    } else {
        putText(img, std::to_string(angle3), p3, font, 1, {0, 0, 0}, 2);
        return 0;
    }
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

void movDron(Mat camara, int angulo, int movimiento) {
    if (movimiento == 1) {
        putText(camara, "Left", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, {0, 0, 0}, 2);

        switch (angulo) {
            //Yaw
            case 1:
                if (anguloYaw > -90) {
                    if (activado == true) {
                        anguloYaw = anguloYaw - 1;
                        microDron->setSetpoints(0, 0, anguloYaw, 0.55);
                    }
                }

                break;

                //Roll
            case 2:
                if (activado == true) {
                    microDron->setSetpoints(0, -20, anguloYaw, 0.55);
                }
                break;
                //Pitch
            case 3:
                if (activado == true) {
                    microDron->setSetpoints(-20, 0, anguloYaw, 0.55);
                }

                break;

            default:
                break;
        }
    } else if (movimiento == 2) {
        putText(camara, "Right", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, {0, 0, 0}, 2);

        switch (angulo) {
            //Yaw
            case 1:
                if (anguloYaw < 90) {
                    if (activado == true) {
                        anguloYaw = anguloYaw + 1;
                        microDron->setSetpoints(0, 0, anguloYaw, 0.55);
                    }
                }

                break;

                //Roll
            case 2:
                if (activado == true) {
                    microDron->setSetpoints(0, 20, anguloYaw, 0.55);
                }
                break;

                //Pitch
            case 3:
                if (activado == true) {
                    microDron->setSetpoints(20, 0, anguloYaw, 0.55);
                }
                break;

            default:
                break;
        }
    } else {
        putText(camara, "Hover", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, {0, 0, 0}, 2);
        if (activado == true) {
            microDron->setSetpoints(0, 0, anguloYaw, 0.55);
        }
    }
    putText(camara, "Angulo", Point(0, 100), FONT_HERSHEY_SIMPLEX, 1, {0, 0, 0}, 2);
    putText(camara, std::to_string(anguloYaw), Point(120, 100), FONT_HERSHEY_SIMPLEX, 1, {0, 0, 0}, 2);

}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
Mat src, cameraOut, dst, src_gray, canny_output, drawing_out;
RNG rng(12345);
vector<vector<Point> > contours;
bool calabaza, calavera, fantasma, murcielago = false;
double hu[7];
double area[1000];

/// Function header
void thresh_callback(int, void *);

Rect getFaceRect(Mat input);

String faceClassifierFileName = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
CascadeClassifier faceCascadeClassifier;

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


void removeFaces(Mat input, Mat output) {
    vector<Rect> faces;
    Mat frameGray;

    cvtColor(input, frameGray, CV_BGR2GRAY);
    equalizeHist(frameGray, frameGray);

    faceCascadeClassifier.detectMultiScale(frameGray, faces, 1.2, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(min_face_size, 20),
                                           Size(max_face_size, 500));
    for (size_t i = 0; i < faces.size(); i++) {
        rectangle(
                output,
                Point(faces[i].x, faces[i].y - 40),
                Point(faces[i].x + faces[i].width + 20, faces[i].y + faces[i].height + 150),
                Scalar(0, 0, 0), -1);
    }
}

Rect getFaceRect(Mat input) {
    vector<Rect> faceRectangles;
    Mat inputGray;

    cvtColor(input, inputGray, CV_BGR2GRAY);
    equalizeHist(inputGray, inputGray);

    faceCascadeClassifier.detectMultiScale(inputGray, faceRectangles, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE,
                                           Size(min_face_size, min_face_size), Size(max_face_size, max_face_size));

    if (faceRectangles.size() > 0)
        return faceRectangles[0];
    else
        return Rect(0, 0, 1, 1);
}


//////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
/***MAIN****/
////////////////////////////////////////////////////////////

void initProyectoIntegrador() {
    camera.set(CV_CAP_PROP_SETTINGS, 1);
    camera.open(0);
    Vec3b intensidadP;
    Vec3b intensidadN;
    Vec3b intensidadO;
    Vec3b intensidadS;
    Vec3b intensidadE;
    Point coordinate;
    /// Load source image

    if (!faceCascadeClassifier.load(faceClassifierFileName))
        throw runtime_error("can't load file " + faceClassifierFileName);

}


int proyectoIntegrador() {
    Mat draw = imread("imagen.jpg", 1);
    calabaza = false;
    calavera = false;
    fantasma = false;
    murcielago = false;
    Mat handMask, foreground;
    int movimiento = 0;

    if (!camera.isOpened()) {
        return -1;
    }
    camera >> dst;

    if(dst.size().width == 0 | dst.size().height == 0){
        return -1;
    }

    cv::resize(dst, src, cv::Size(640, 360));

    cameraOut = src.clone();

    drawSkinColorSampler(cameraOut);

    foreground = getForeground(src);

    handMask = getSkinMask(foreground);

    removeFaces(src, handMask);


    if (shouldCalibrate_) {
        calibrate(src);
    } // s
    else if (shouldCalibrateB_) {
        calibrateB(src);
    } //b
    else if (shouldActivate_) {
        activado = true;
    } else if (shouldStop_) {
        activado = false;
        putText(cameraOut, "KILL MOTORS", Point(0, 50), FONT_HERSHEY_SIMPLEX, 1, {0, 0, 0}, 2);

    }

    /// Create Window
    const char *source_window = "Source";

    /// Convert image to gray and blur it
    blur(handMask, handMask, Size(3, 3));

    //imshow("Mascara",handMask);

    /// Detect edges using canny
    Canny(handMask, canny_output, 10, 30, 3, true);

    /// Se vuelve a hacer borrosa imagen.
    blur(canny_output, canny_output, Size(4, 4));

    /// Find contours
    findContours(canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    /// Get the moments
    vector<Moments> mu(contours.size());
    vector<Moments> mu2(contours.size());
    vector<Point2f> mc(contours.size());
    vector<Point2f> mc2(contours.size());

    drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    vector<vector<Point> > hull(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {

        mu2[i] = moments(contours[i]);

        //Se obtiene el ConvexHULL de los contornos.
        convexHull(Mat(contours[i]), hull[i], false);
        mu[i] = moments(hull[i]);
        area[i] = contourArea(hull[i]);

        ///  Centros de masa para Contours y ConvexHUll.
        //add 1e-5 to avoid division by zero
        mc[i] = Point2f(static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
                        static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)));

        mc2[i] = Point2f(static_cast<float>(mu2[i].m10 / (mu2[i].m00 + 1e-5)),
                         static_cast<float>(mu2[i].m01 / (mu2[i].m00 + 1e-5)));

        //Se obtienen los momentos HU de contornos de ConvexHull.
        HuMoments(mu[i], hu);

        //Solo toma en consideracion el area mas grande de los contornos.
        if (area[i] > 6000) {

            Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

            //YAW
            if ((hu[0] > 0.144755964179105 && hu[0] < 0.315057098507463 && hu[1] > 0.000028524928631841 &&
                 hu[1] < 0.00205005761512415)) {

                //Area to small, KILL MOTORS
                if (area[i] < 15000) {
                    drawContours(drawing, contours, (int) i, {0, 255, 0}, CV_FILLED);
                    putText(cameraOut, "KILL MOTORS", Point(0, 50), FONT_HERSHEY_SIMPLEX, 1, {0, 0, 0}, 2);
                    microDron->emergencyStop();
                } else {
                    drawContours(drawing, contours, (int) i, {0, 0, 255}, CV_FILLED);
                    movimiento = drawLine(cameraOut, mu2[i], mc2[i], "Yaw: ");
                    movDron(cameraOut, 1, movimiento);
                }
                circle(cameraOut, mc[i], 3, color, -1);
            }

            //ROLL
            if (hu[0] > 0.149414392707118 && hu[0] < 0.390490148833139 && hu[1] > 0.0100210477596266 &&
                hu[1] < 0.0348693392415403) {
                /// Draw contours
                Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                drawContours(drawing, contours, (int) i, {26, 127, 239}, CV_FILLED);
                circle(src, mc[i], 3, color, -1);
                movimiento = drawLine(cameraOut, mu[i], mc[i], "Roll: ");
                movDron(cameraOut, 2, movimiento);
            }

            //PITCH
            if (hu[0] > 0.151857713205418 && hu[0] < 0.205454553160271 && hu[1] > 0.00205005761512415 &&
                hu[1] < 0.00826184265575621) {

                //Area is to small, KILL MOTORS
                if (area[i] < 14000) {
                    drawContours(drawing, contours, (int) i, {0, 255, 0}, CV_FILLED);
                    putText(cameraOut, "KILL MOTORS", Point(0, 50), FONT_HERSHEY_SIMPLEX, 1, {0, 0, 0}, 2);
                    microDron->emergencyStop();
                }
                    //DRAW PITCH LINE
                else {
                    drawContours(drawing, contours, (int) i, {240, 180, 12}, CV_FILLED);
                    movimiento = drawLine(cameraOut, mu[i], mc[i], "Pitch: ");
                    movDron(cameraOut, 3, movimiento);
                }
                circle(src, mc[i], 3, color, -1);
            }
        }
    }
    return 1;
}

void setMicroDronInterface(MicroDronInterface &interface) {
    microDron = &interface;
}

const Mat &getVisionOutput() {
    return cameraOut;
}

const Mat &getHandMask() {
    return canny_output;
}

const Mat &getContours() {
    return drawing;
}

void shouldCalibrateA(bool should) {
    shouldCalibrate_ = should;
}

void shouldCalibrateB(bool should) {
    shouldCalibrateB_ = should;
}

void shouldActivate(bool should) {
    shouldActivate_ = should;
}

void shouldStop(bool should) {
    shouldStop_ = should;
}

void stopProyectoIntegrador() {
    camera.release();
}
