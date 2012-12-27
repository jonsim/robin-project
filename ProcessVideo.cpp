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
#define SERVER_PORT_1 1401  // Port to listen for streaming depth  data on.
#define SERVER_PORT_2 1402  // Port to listen for streaming colour data on.

// Macros
#define CHECK_RETURN(r, what)       \
    if (r < 0)                      \
    {                               \
        perror(what);               \
        return 1;                   \
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

int main (void)
{
    // Create the server
    struct    sockaddr_in server_address;
    struct    sockaddr_in client_address;
    socklen_t client_address_length = sizeof(client_address);
    int server_socket, client_socket;
    int bytes_read;
    int retVal;
    char buffer [128];

    printf("Setting up the TCP server... ");
    fflush(stdout);

    // Create an IP streaming socket (which uses TCP).
    server_socket = socket(PF_INET, SOCK_STREAM, 0);
    CHECK_RETURN(server_socket, "socket");

    // Bind to port.
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port   = htons(SERVER_PORT_1);     // host to network short: converts a u_short from host to TCP network byte order (big-endian).
    retVal = bind(server_socket, (struct sockaddr*) &server_address, sizeof(server_address));
    CHECK_RETURN(retVal, "bind");

    // Set to listen on the port with a connection queue length of 1.
    retVal = listen(server_socket, 1);
    CHECK_RETURN(retVal, "listen");
    
    printf("done.\n");
    
    // Wait until a client connects to us.
    printf("Waiting for TCP connection on port %d.\n", SERVER_PORT_1);
    client_socket = accept(server_socket, (struct sockaddr*) &client_address, &client_address_length);
    CHECK_RETURN(client_socket, "accept");
    printf("TCP connection from %s:%d\n", inet_ntoa(client_address.sin_addr), htons(client_address.sin_port));
    
    // Transfer data with the client.
    while (1)
    {
        while ((bytes_read = read(client_socket, buffer, sizeof(buffer))) > 0)
        {
            printf ("Received %d bytes.\n", bytes_read);
        }
        CHECK_RETURN(bytes_read, "read");
    }
    
    // Finish up.
    close(client_socket);
    close(server_socket);
    
    return 0 ;
    
    
    
    
    

    // Create the images
    IplImage* imgDepth = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_16U, 1);
    IplImage* imgColor = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
    
    
    
    /*
    // Read the files into the images
    FILE* f;
    f = fopen("fc_640x480_d.dat", "rb");
    fread(imgDepth->imageData, 2, IMAGE_WIDTH*IMAGE_HEIGHT, f);
    fclose(f);
    f = fopen("fc_640x480_c.dat", "rb");
    fread(imgColor->imageData, 1, IMAGE_WIDTH*IMAGE_HEIGHT*3, f);
    cvCvtColor(imgColor, imgColor, CV_RGB2BGR);     // convert from OpenNIs RGB to OpenCVs BGR
    fclose(f);
    */
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
