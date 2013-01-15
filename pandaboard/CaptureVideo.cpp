/****************************************************************************
*                                                                           *
*  OpenNI 1.x Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
// networking
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
// openni
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>
// opencv
#include <cv.h>
#include <highgui.h>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../../Config/SamplesConfig.xml"
#define SAMPLE_XML_PATH_LOCAL "/home/jon/kinect/project/SamplesConfig.xml"
#define SERVER_ADDRESS "192.168.0.4"
#define SERVER_PORT_D 1401
#define SERVER_PORT_C 1402
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
#define DEPTH_STREAM
//#define COLOR_STREAM
#if COLOR_STREAM
    #error "NO COLOUR STREAM PLESE :(... or write it yourself"
#endif

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)                                              \
    if (rc != XN_STATUS_OK)                                             \
    {                                                                   \
        printf("%s failed: %s\n", what, xnGetStatusString(rc));         \
        return rc;                                                      \
    }
#define CHECK_RETURN(r, what)       \
    if (r < 0)                      \
    {                               \
        perror(what);               \
        exit(EXIT_FAILURE);         \
    }

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace xn;


void createColourDepthImage (cv::Mat* dst, const XnDepthPixel* src)
{
    static const float scaling_factor = 4800/1530;
    uint16_t x=0, y=0;
    uint32_t y_offset=0, total_offset=0;
    uint16_t v;
    uint8_t  r, g, b;
    uint8_t* loc;
    
    for (y = 0, total_offset = 0; y < IMAGE_HEIGHT; y++)
    {
        for (x = 0; x < IMAGE_WIDTH; x++, total_offset++)
        {
            v = (uint16_t) src[total_offset];
            
            if (v == 0)
            {
                r = 0;
                g = 0;
                b = 0;
            }
            else
            {
                v -= 400;
                v = (v < 4800) ? v / scaling_factor : 1530;
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
                else  // v <= 1530
                {
                    r = 255u;
                    g =   0u;
                    b = (uint8_t) (1530 - v);  // b falls to 0
                }
            }
            
            loc = dst->data + total_offset;
            loc[0] = b;
            loc[1] = g;
            loc[2] = r;
        }
    }
}



XnBool fileExists(const char *fn)
{
    XnBool exists;
    xnOSDoesFileExist(fn, &exists);
    return exists;
}

void setupTCPClient (int* clientSocket, struct sockaddr_in* serverAddress, int serverPort)
{
    int retVal;
    
    // Create a TCP socket.
    (*clientSocket) = socket(PF_INET, SOCK_STREAM, 0);
    CHECK_RETURN((*clientSocket), "socket");
    
    // Set up the address of the server as given by the defines.
	// Note that both address and port number must be in network byte order.
	memset(serverAddress, 0, sizeof((*serverAddress)));
	serverAddress->sin_family = AF_INET;
	serverAddress->sin_port   = htons(serverPort);
	retVal = inet_aton(SERVER_ADDRESS, &(serverAddress->sin_addr));
	CHECK_RETURN(retVal, "inet_aton");      // NB: There was originally a close(client_socket) in this if.
}

void connectToServer (int* clientSocket, struct sockaddr_in* serverAddress, int serverPort, ScriptNode* scriptNode, Context* context)
{
    int retVal;
    
    retVal = connect((*clientSocket), (struct sockaddr*) serverAddress, sizeof((*serverAddress)));
    if (retVal < 0)
    {
        printf("Failed to connect to %s:%d, exitting.\n", SERVER_ADDRESS, serverPort);
        scriptNode->Release();
        context->Release();
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("Connected to %s:%d.\n", SERVER_ADDRESS, serverPort);
    }
}

int main()
{
    XnStatus nRetVal = XN_STATUS_OK;

    Context context;
    ScriptNode scriptNode;
    EnumerationErrors errors;

    // Load config file
    printf("Reading config file... ");
    fflush(stdout);
    const char *fn = NULL;
    if (fileExists(SAMPLE_XML_PATH))
        fn = SAMPLE_XML_PATH;
    else if (fileExists(SAMPLE_XML_PATH_LOCAL))
        fn = SAMPLE_XML_PATH_LOCAL;
    else
    {
        printf("Could not find '%s' nor '%s'. Aborting.\n" , SAMPLE_XML_PATH, SAMPLE_XML_PATH_LOCAL);
        return XN_STATUS_ERROR;
    }
    nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);

    if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
    {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (nRetVal);
    }
    CHECK_RC(nRetVal, "Open");

    printf("done.\n");


    // main variables (openni)
    DepthGenerator depth;
    ImageGenerator color;
    DepthMetaData  depthMD;
    ImageMetaData  colorMD;
    XnFPSData      xnFPS;
    // main variables (opencv)
    cv::Mat colouredDepthImage(  IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    cv::Mat anotherDepthImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);


    // initialise and validate main variables
    printf("Initialising camera components... ");
    fflush(stdout);
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
    CHECK_RC(nRetVal, "Find depth generator");
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, color);
    CHECK_RC(nRetVal, "Find image generator");
    nRetVal = xnFPSInit(&xnFPS, 180);
    CHECK_RC(nRetVal, "FPS Init");
    printf("done.\n");
    
    
    // Setup server
#ifdef DEPTH_STREAM
	struct sockaddr_in serverAddressD;
    int clientSocketD;
#endif
#ifdef COLOR_STREAM
	struct sockaddr_in serverAddressC;
    int clientSocketC;
#endif
    int retVal;
    
    printf("Setting up the TCP client... ");
    fflush(stdout);
#ifdef DEPTH_STREAM
    setupTCPClient(&clientSocketD, &serverAddressD, SERVER_PORT_D);
#endif
#ifdef COLOR_STREAM
    setupTCPClient(&clientSocketC, &serverAddressC, SERVER_PORT_C);
#endif
    printf("done.\n");
    
    // Connect to the server.
    printf("Connecting to server...\n");
#ifdef DEPTH_STREAM
    connectToServer(&clientSocketD, &serverAddressD, SERVER_PORT_D, &scriptNode, &context);
    const int depthBufferLen = sizeof(XnDepthPixel) * 640 * 480;
#endif
#ifdef COLOR_STREAM
    connectToServer(&clientSocketC, &serverAddressC, SERVER_PORT_C, &scriptNode, &context);
    const int colorBufferLen = sizeof(XnRGB24Pixel) * 640 * 480;
#endif

    while (!xnOSWasKeyboardHit())
    {
        // Update the data.
        nRetVal = context.WaitAndUpdateAll();
        if (nRetVal != XN_STATUS_OK)
        {
            printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
            continue;
        }

        // FPS
        xnFPSMarkFrame(&xnFPS);
        printf("FPS: %.1f\r", xnFPSCalc(&xnFPS));
        fflush(stdout);

        // Read the data into our containers.
#ifdef DEPTH_STREAM
        depth.GetMetaData(depthMD);
        const XnDepthPixel* depthData = depthMD.Data();
        std::vector<uint8_t> buffer;
        
        createColourDepthImage(&colouredDepthImage, depthData);
        cv::imencode(".JPEG", colouredDepthImage, buffer);
        anotherDepthImage = cv::imdecode(buffer, 1);
        
        retVal = write(clientSocketD, depthData, depthBufferLen);
        if (retVal < 0)
            break;
#endif
#ifdef COLOR_STREAM
        color.GetMetaData(colorMD);
        const XnRGB24Pixel* colorData = colorMD.RGB24Data();
        retVal = write(clientSocketC, colorData, colorBufferLen);
        if (retVal < 0)
            break;
#endif
    }

    // Finish up.
    printf("\nExitting.\n");
#ifdef DEPTH_STREAM
    close(clientSocketD);
    depth.Release();
#endif
#ifdef COLOR_STREAM
    close(clientSocketC);
    color.Release();
#endif
    scriptNode.Release();
    context.Release();

    return 0;
}
