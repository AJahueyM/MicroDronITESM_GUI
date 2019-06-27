
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"

#include <iostream>
#include <iomanip>

using namespace cv;
using namespace std;

void initProyectoIntegrador();
int proyectoIntegrador();
void stopProyectoIntegrador();

const Mat& getVisionOutput();
const Mat& getHandMask();
const Mat& getContours();
void setMicroDronInterface(MicroDronInterface& interface);

void shouldCalibrateA(bool should);
void shouldCalibrateB(bool should);
void shouldActivate(bool should);
void shouldStop(bool should);
