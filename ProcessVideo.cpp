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

// Macros
#define CHECK_RETURN(r, what)       \
    if (r < 0)                      \
    {                               \
        perror(what);               \
        exit(EXIT_FAILURE);         \
    }


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
    
//    return 0;
}

void waitForClientConnection (int* serverSocket, int* clientSocket, struct sockaddr_in* clientAddress, socklen_t* clientAddressLength)
{
    (*clientSocket) = accept((*serverSocket), (struct sockaddr*) clientAddress, clientAddressLength);
    CHECK_RETURN((*clientSocket), "accept");
    printf("TCP connection from %s:%d\n", inet_ntoa(clientAddress->sin_addr), htons(clientAddress->sin_port));
    
//    return 0;
}

int main (void)
{
    // Create the images
    printf("Creating the images... ");
    fflush(stdout);
    IplImage* imgDepth = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_16U, 1);
    IplImage* imgColor = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
    cvNamedWindow("DepthVideo", CV_WINDOW_AUTOSIZE);
    printf("done.\n");
    

    // Create the server
    struct    sockaddr_in serverAddressD;
    struct    sockaddr_in serverAddressC;
    struct    sockaddr_in clientAddressD;
    struct    sockaddr_in clientAddressC;
    socklen_t clientAddressLength = sizeof(clientAddressD);
    int serverSocketD, serverSocketC, clientSocketD, clientSocketC;
    int bytes_read;
    int total_bytes_read;
    int retVal;

    printf("Setting up the TCP server... ");
    fflush(stdout);
    setupTCPServer(&serverSocketD, &serverAddressD, SERVER_PORT_D, &clientSocketD, &clientAddressD);
    printf("done.\n");
    
    // Wait until a client connects to us.
    printf("Waiting for TCP connection on port %d.\n", SERVER_PORT_D);
    waitForClientConnection(&serverSocketD, &clientSocketD, &clientAddressD, &clientAddressLength);
    
    // Transfer data with the client.
    total_bytes_read = 0;
    const int depth_buffer_len = 2 * IMAGE_WIDTH * IMAGE_HEIGHT;
    const int color_buffer_len = 3 * IMAGE_WIDTH * IMAGE_HEIGHT;
    int buttonPress;
    while (1)
    {
        //while ((bytes_read = read(client_socket, buffer, sizeof(buffer))) > 0)
        while ((bytes_read = read(clientSocketD, (imgDepth->imageData + total_bytes_read), (depth_buffer_len - total_bytes_read))) > 0)
        {
            total_bytes_read += bytes_read;
            //printf ("Received %d bytes (%d/320000).\n", bytes_read, total_bytes_read);
            // Check if the buffer is full
            if (total_bytes_read >= depth_buffer_len)
            {
                total_bytes_read = 0;
                cvScale(imgDepth, imgDepth, 12);
                cvShowImage("DepthVideo", imgDepth);
                buttonPress = cvWaitKey(5);
                if (buttonPress >= 0)
                    goto loop_exit;
            }
        }
        CHECK_RETURN(bytes_read, "read");
        
        // DO SHIT HERE
        
        /*// Read the files into the images
        FILE* f;
        f = fopen("fc_640x480_d.dat", "rb");
        fread(imgDepth->imageData, 2, IMAGE_WIDTH*IMAGE_HEIGHT, f);
        fclose(f);
        f = fopen("fc_640x480_c.dat", "rb");
        fread(imgColor->imageData, 1, IMAGE_WIDTH*IMAGE_HEIGHT*3, f);
        cvCvtColor(imgColor, imgColor, CV_RGB2BGR);     // convert from OpenNIs RGB to OpenCVs BGR
        fclose(f);
        *//*
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
        //cvSaveImage("DifferenceImage.png", diffImage);*/
    } loop_exit:
    
    printf("exitting.\n");
    // Finish up. Currently this section can never be reached.
    close(clientSocketD);
    close(serverSocketD);
    //close(clientSocketC);
    //close(serverSocketC);

    // Wait until key pressed
    //cvWaitKey();

    // Release memory and cleanup
    cvDestroyAllWindows();
    cvReleaseImage(&imgDepth);
    cvReleaseImage(&imgColor);
    
    return 0;
}
