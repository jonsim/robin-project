// Includes
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "/usr/include/opencv/cxcore.h"
#include "/usr/include/opencv/cv.h"
#include "/usr/include/opencv/highgui.h"

// Defines
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
#define SERVER_PORT_D 1401  // Port to listen for streaming depth  data on.
#define SERVER_PORT_C 1402  // Port to listen for streaming colour data on.
#define DEPTH_STREAM
//#define COLOR_STREAM
#define FILTER_SIZE 2

// Macros
#define CHECK_RETURN(r, what)       \
    if (r < 0)                      \
    {                               \
        perror(what);               \
        exit(EXIT_FAILURE);         \
    }


/********** FUNCTIONS **********/
//returns value between 0 and 255 of pixel at image position (x,y)
inline uint8_t getPixel_SB (IplImage* img, const uint16_t x, const uint16_t y)
{
    return ((uint8_t*) (img->imageData + img->widthStep*y))[img->nChannels*x];
}
inline uint16_t getPixel_DB (IplImage* img, const uint16_t x, const uint16_t y)
{
    return ((uint16_t*) (img->imageData + img->widthStep*y))[img->nChannels*x];
}

//sets pixel at image position (x,y)
inline void setPixel_SB (IplImage* img, const uint16_t x, const uint16_t y, uint8_t v)
{
    ((uint8_t*) (img->imageData + img->widthStep*y))[x*img->nChannels] = v;
}
inline void setPixel_DB (IplImage* img, const uint16_t x, const uint16_t y, uint16_t v)
{
    ((uint16_t*) (img->imageData + img->widthStep*y))[img->nChannels*x] = v;
}
inline void setPixels_SB (IplImage* img, const uint16_t x, const uint16_t y, const uint8_t r, const uint8_t g, const uint8_t b)
{
    ((uint8_t*) (img->imageData + img->widthStep*y))[x*img->nChannels + 0] = b;
    ((uint8_t*) (img->imageData + img->widthStep*y))[x*img->nChannels + 1] = g;
    ((uint8_t*) (img->imageData + img->widthStep*y))[x*img->nChannels + 2] = r;
}
inline void setPixel_depthColour (IplImage* dst, IplImage* src, const uint16_t x, const uint16_t y)
{
    // get v and then cast it to be between 0 and 255*3=765 when v's max is assumed to be 8000
    uint8_t r, g, b;
    uint16_t v = getPixel_DB(src, x, y);
    
    if (v == 0)
    {
        r = 0;
        g = 0;
        b = 0;
    }
    else
    {
        v = v - 400;
        v = v / (4800 / 1530);
        // H' takes values between 0-1530
        // H' =    0- 255  RGB=   255, 0-255, 0
        // H' =  255- 510  RGB= 255-0,   255, 0
        // H' =  510- 765  RGB=     0,   255, 0-255
        // H' =  765-1020  RGB=     0, 255-0, 255
        // H' = 1020-1275  RGB= 0-255,     0, 255
        // H' = 1275-1530  RGB=   255,     0, 255-0
        if (v < 255)
        {
            r = 255u;
            g = (uint8_t) v;  // g increases to 255
            b =   0u;
        }
        else if (v < 510)
        {
            r = (uint8_t) (510 - v);  // r falls to 0
            g = 255u;
            b =   0u;
        }
        else if (v < 765)
        {
            r =   0u;
            g = 255u;
            b = (uint8_t) (v - 510);  // b increases to 255
        }
        else if (v < 1020)
        {
            r =   0u;
            g = (uint8_t) (1020 - v);  // g falls to 0
            b = 255u;
        }
        else if (v < 1275)
        {
            r = (uint8_t) (v - 1020);  // r increases to 255
            g =   0u;
            b = 255u;
        }
        else  // v < 1530
        {
            r = 255u;
            g =   0u;
            b = (uint8_t) (1530 - v);  // b falls to 0
        }
    }
    
    setPixels_SB(dst, x, y, r, g, b);
}

// Retrieves the average pixel value in a neighbourhood of a given size around the given pixel.
uint16_t calculateNeighbourhoodAverage_DB (IplImage* img, const uint16_t centre_x, const uint16_t centre_y, const uint8_t size)
{
    uint16_t min_x = (centre_x < size)              ? 0            : centre_x - size;
    uint16_t max_x = (centre_x > IMAGE_WIDTH-size)  ? IMAGE_WIDTH  : centre_x + size;
    uint16_t min_y = (centre_y < size)              ? 0            : centre_y - size;
    uint16_t max_y = (centre_y > IMAGE_HEIGHT-size) ? IMAGE_HEIGHT : centre_y + size;
    uint16_t pixel_value;
    uint32_t total = 0;
    uint32_t pixel_count = 0;
    
    for (uint16_t y = min_y; y < max_y; y++)
    {
        for (uint16_t x = min_x; x < max_x; x++)
        {
            pixel_value = getPixel_DB(img, x, y);
            if (pixel_value != 0)
            {
                pixel_count++;
                total += pixel_value;
            }
        }
    }
    
    if (pixel_count == 0)
        return 0;
    return (uint16_t) total / pixel_count;
}


void setupTCPServer (int* serverSocket, struct sockaddr_in* serverAddress, int serverPort, int* clientSocket, struct sockaddr_in* clientAddress)
{
    int retVal;
    
    // Create an IP streaming socket (which uses TCP).
    (*serverSocket) = socket(PF_INET, SOCK_STREAM, 0);
    CHECK_RETURN((*serverSocket), "socket");

    // Bind to port.
    memset(serverAddress, 0, sizeof((*serverAddress)));
    serverAddress->sin_family = AF_INET;
    serverAddress->sin_port   = htons(serverPort);     // host to network short: converts a u_short from host to TCP network byte order (big-endian).
    retVal = bind((*serverSocket), (struct sockaddr*) serverAddress, sizeof((*serverAddress)));
    CHECK_RETURN(retVal, "bind");

    // Set to listen on the port with a connection queue length of 1.
    retVal = listen((*serverSocket), 1);
    CHECK_RETURN(retVal, "listen");
}

void waitForClientConnection (int* serverSocket, int* clientSocket, struct sockaddr_in* clientAddress, socklen_t* clientAddressLength)
{
    (*clientSocket) = accept((*serverSocket), (struct sockaddr*) clientAddress, clientAddressLength);
    CHECK_RETURN((*clientSocket), "accept");
    printf("TCP connection from %s:%d.\n", inet_ntoa(clientAddress->sin_addr), htons(clientAddress->sin_port));
}

int main (void)
{
    // Create the images
    printf("Creating the images... ");
    fflush(stdout);
#ifdef DEPTH_STREAM
    IplImage* imgDepthIn    = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_16U, 1);
    IplImage* imgDepthIn_P1 = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_16U, 1);
    IplImage* imgDepthIn_P2 = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_16U, 1);
    IplImage* imgDepthOut   = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_16U, 1);
    IplImage* imgDepthDisp  = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
#endif
#ifdef COLOR_STREAM
    IplImage* imgColor = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
#endif
    printf("done.\n");
    

    // Create the server
#ifdef DEPTH_STREAM
    struct    sockaddr_in serverAddressD;
    struct    sockaddr_in clientAddressD;
    socklen_t clientAddressLengthD = sizeof(clientAddressD);
    int serverSocketD;
    int clientSocketD;
    int totalBytesReadD;
#endif
#ifdef COLOR_STREAM
    struct    sockaddr_in serverAddressC;
    struct    sockaddr_in clientAddressC;
    socklen_t clientAddressLengthC = sizeof(clientAddressC);
    int serverSocketC;
    int clientSocketC;
    int totalBytesReadC;
#endif
    int bytesRead;
    //int retVal;

    printf("Setting up the TCP server... ");
    fflush(stdout);
#ifdef DEPTH_STREAM
    setupTCPServer(&serverSocketD, &serverAddressD, SERVER_PORT_D, &clientSocketD, &clientAddressD);
#endif
#ifdef COLOR_STREAM
    setupTCPServer(&serverSocketC, &serverAddressC, SERVER_PORT_C, &clientSocketC, &clientAddressC);
#endif
    printf("done.\n");
    
    // Wait until a client connects to us.
#ifdef DEPTH_STREAM
    printf("Waiting for TCP connection on port %d.\n", SERVER_PORT_D);
    waitForClientConnection(&serverSocketD, &clientSocketD, &clientAddressD, &clientAddressLengthD);
#endif
#ifdef COLOR_STREAM
    printf("Waiting for TCP connection on port %d.\n", SERVER_PORT_C);
    waitForClientConnection(&serverSocketC, &clientSocketC, &clientAddressC, &clientAddressLengthC);
#endif
    
    // Transfer data with the client.
#ifdef DEPTH_STREAM
    //cvNamedWindow("DepthVideo", CV_WINDOW_AUTOSIZE);
    totalBytesReadD = 0;
    const int depthBufferLen = 2 * IMAGE_WIDTH * IMAGE_HEIGHT;
    uint32_t frameNumber = 0;
#endif
#ifdef COLOR_STREAM
    //cvNamedWindow("ColorVideo", CV_WINDOW_AUTOSIZE);
    totalBytesReadC = 0;
    const int colorBufferLen = 3 * IMAGE_WIDTH * IMAGE_HEIGHT;
#endif
    int buttonPress;
    
    printf("Connecting completed, receiving data.\n");
    while (1)
    {
        // read from the depth connection
#ifdef DEPTH_STREAM
        while ((bytesRead = read(clientSocketD, (imgDepthIn->imageData + totalBytesReadD), (depthBufferLen - totalBytesReadD))) > 0)
        {
            totalBytesReadD += bytesRead;
            // Check if the buffer is full
            if (totalBytesReadD >= depthBufferLen)
            {
                // house keeping
                totalBytesReadD = 0;
                frameNumber++;
                
                // noise removal
                //cvScale(imgDepthIn, imgDepthIn, 12);
                //memcpy(imgDepthOut->imageData, imgDepthIn->imageData, depthBufferLen);
                for (uint16_t y = 0; y < IMAGE_HEIGHT; y++)
                {
                    for (uint16_t x = 0; x < IMAGE_WIDTH; x++)
                    {
                        setPixel_depthColour(imgDepthDisp, imgDepthIn, x, y);
                    }
                }
                cvScale(imgDepthIn, imgDepthIn, 12);
                /*if (frameNumber > 2)
                {
                    uint16_t vPrev1, vPrev2, vCurr, vNew;
                    for (uint16_t y = 0; y < IMAGE_HEIGHT; y++)
                    {
                        for (uint16_t x = 0; x < IMAGE_WIDTH; x++)
                        {
                            vCurr  = getPixel_DB(imgDepthIn,    x, y);
                            vPrev1 = getPixel_DB(imgDepthIn_P1, x, y);
                            //vPrev2 = getPixel_DB(imgDepthIn_P2, x, y);
                            if (vCurr == 0)
                            {
                                vNew = vPrev1;
                                //vNew = calculateNeighbourhoodAverage_DB(imgDepthIn, x, y, FILTER_SIZE);
                                //vNew = vPrev + ((vCurr - vPrev) / 2);
                                setPixel_DB(imgDepthOut, x, y, vNew);
                            }
                        }
                    }
                }*/
                //memcpy(imgDepthIn_P2->imageData, imgDepthIn_P1->imageData, depthBufferLen);
                //memcpy(imgDepthIn_P1->imageData, imgDepthIn->imageData,    depthBufferLen);
                
                // scale + show image
                cvShowImage("Raw Depth Video", imgDepthIn);
                cvShowImage("Corrected Depth Video", imgDepthDisp);
                buttonPress = cvWaitKey(5);
                
                // check for exit conditions
                if (buttonPress >= 0)
                    goto loop_exit;
                break;  // give the other stream a chance to catch up.
            }
        }
        CHECK_RETURN(bytesRead, "read_depth");
#endif
      
#ifdef COLOR_STREAM
        while ((bytesRead = read(clientSocketC, (imgColor->imageData + totalBytesReadC), (colorBufferLen - totalBytesReadC))) > 0)
        {
            totalBytesReadC += bytesRead;
            //printf ("Received %d bytes (%d/320000).\n", bytes_read, total_bytes_read);
            // Check if the buffer is full
            if (totalBytesReadC >= colorBufferLen)
            {
                totalBytesReadC = 0;
                cvCvtColor(imgColor, imgColor, CV_RGB2BGR);     // convert from OpenNIs RGB to OpenCVs BGR
                cvShowImage("ColorVideo", imgColor);
                buttonPress = cvWaitKey(5);
                if (buttonPress >= 0)
                    goto loop_exit;
                break;
            }
        }
        CHECK_RETURN(bytesRead, "read_color");
#endif
    } loop_exit:
    
    printf("Exitting.\n");
    
    // Finish up
    cvDestroyAllWindows();
#ifdef DEPTH_STREAM
    close(clientSocketD);
    close(serverSocketD);
    cvReleaseImage(&imgDepthIn);
    cvReleaseImage(&imgDepthIn_P1);
    cvReleaseImage(&imgDepthIn_P2);
    cvReleaseImage(&imgDepthOut);
#endif
#ifdef COLOR_STREAM
    close(clientSocketC);
    close(serverSocketC);
    cvReleaseImage(&imgColor);
#endif
    
    return 0;
}
