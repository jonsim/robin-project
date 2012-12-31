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

// Macros
#define CHECK_RETURN(r, what)       \
    if (r < 0)                      \
    {                               \
        perror(what);               \
        exit(EXIT_FAILURE);         \
    }


/********** FUNCTIONS **********/
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
    IplImage* imgDepth = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_16U, 1);
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
    int retVal;

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
    cvNamedWindow("DepthVideo", CV_WINDOW_AUTOSIZE);
    totalBytesReadD = 0;
    const int depthBufferLen = 2 * IMAGE_WIDTH * IMAGE_HEIGHT;
#endif
#ifdef COLOR_STREAM
    cvNamedWindow("ColorVideo", CV_WINDOW_AUTOSIZE);
    totalBytesReadC = 0;
    const int colorBufferLen = 3 * IMAGE_WIDTH * IMAGE_HEIGHT;
#endif
    int buttonPress;
    
    printf("Connecting completed, receiving data.\n");
    while (1)
    {
        // read from the depth connection
#ifdef DEPTH_STREAM
        while ((bytesRead = read(clientSocketD, (imgDepth->imageData + totalBytesReadD), (depthBufferLen - totalBytesReadD))) > 0)
        {
            totalBytesReadD += bytesRead;
            // Check if the buffer is full
            if (totalBytesReadD >= depthBufferLen)
            {
                totalBytesReadD = 0;
                cvScale(imgDepth, imgDepth, 12);
                cvShowImage("DepthVideo", imgDepth);
                buttonPress = cvWaitKey(5);
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
    cvReleaseImage(&imgDepth);
#endif
#ifdef COLOR_STREAM
    close(clientSocketC);
    close(serverSocketC);
    cvReleaseImage(&imgColor);
#endif
    
    return 0;
}
