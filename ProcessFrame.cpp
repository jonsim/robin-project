// Includes
#include <stdio.h>
#include <math.h>
#include "/usr/include/opencv/cxcore.h"
#include "/usr/include/opencv/cv.h"
#include "/usr/include/opencv/highgui.h"

// Defines
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480

// Globals
IplImage* imgDepth;
IplImage* imgColor;

/********** FUNCTIONS **********/
//returns value between 0 and 255 of pixel at image position (x,y)
unsigned char getPixel (IplImage* img, int x, int y)
{
    return ((unsigned char*) (img->imageData + img->widthStep*y))[x * img->nChannels];
}

//sets pixel at image position (x,y)
void setPixel (IplImage* img, int x, int y, unsigned char v)
{
    ((unsigned char*) (img->imageData + img->widthStep*y))[x * img->nChannels] = v;
}

int main (void)
{
    int retVal;
    
    // Create the images
    imgDepth = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_16U, 1);
    imgColor = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
    
    // Read the files into the images
    FILE* f;
    f = fopen("fc_640x480_d.dat", "rb");
    retVal = fread(imgDepth->imageData, 2, IMAGE_WIDTH*IMAGE_HEIGHT, f);
    fclose(f);
    f = fopen("fc_640x480_c.dat", "rb");
    retVal = fread(imgColor->imageData, 1, IMAGE_WIDTH*IMAGE_HEIGHT*3, f);
    cvCvtColor(imgColor, imgColor, CV_RGB2BGR);     // convert from OpenNIs RGB to OpenCVs BGR
    fclose(f);
    
    // Process
    cvScale(imgDepth, imgDepth, 12); // adjust contrast to brighten depth image
    //cvDilate(imgColor, imgColor, 0, 4);
    //cvDilate(imgDepth, imgDepth, 0, 4);
    
    // Display the images
    cvNamedWindow("Color Image", CV_WINDOW_AUTOSIZE);
    cvShowImage("Color Image", imgColor);
    cvNamedWindow("Depth Image", CV_WINDOW_AUTOSIZE);
    cvShowImage("Depth Image", imgDepth);
    //cvSaveImage("SegmentedImage.png", segImage);
    //cvSaveImage("DifferenceImage.png", diffImage);

    // Wait until key pressed
    cvWaitKey();

    // Release memory and cleanup
    cvDestroyAllWindows();
    cvReleaseImage(&imgDepth);
    cvReleaseImage(&imgColor);
    
    return 0;
}
