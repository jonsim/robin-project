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
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>
#include <stdio.h>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../../Config/SamplesConfig.xml"
#define SAMPLE_XML_PATH_LOCAL "SamplesConfig.xml"
#define SERVER_ADDRESS "192.168.159.15"
#define SERVER_PORT_1 1401
#define SERVER_PORT_2 1402

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
        return 1;                   \
    }

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace xn;

XnBool fileExists(const char *fn)
{
    XnBool exists;
    xnOSDoesFileExist(fn, &exists);
    return exists;
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

    // main variables
    DepthGenerator depth;
    ImageGenerator image;
    DepthMetaData  depthMD;
    ImageMetaData  imageMD;
    XnFPSData      xnFPS;

    // initialise and validate main variables
    printf("Initialising camera components... ");
    fflush(stdout);
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
    CHECK_RC(nRetVal, "Find depth generator");
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image);
    CHECK_RC(nRetVal, "Find image generator");
    nRetVal = xnFPSInit(&xnFPS, 180);
    CHECK_RC(nRetVal, "FPS Init");
    printf("done.\n");
    
    
    
    
    // Setup server
	struct sockaddr_in server_address;
    int client_socket;
    //char* server_name = "192.168.159.15";
    char buffer[320000];
    int buffer_len = 320000;
    int retVal;
    
    printf("Setting up the TCP client... ");
    fflush(stdout);
    
    // Create a TCP socket.
    client_socket = socket(PF_INET, SOCK_STREAM, 0);
    CHECK_RETURN(client_socket, "socket");
    
    // Set up the address of the server as given by the defines.
	// Note that both address and port number must be in network byte order.
	memset(&server_address, 0, sizeof(server_address));
	server_address.sin_family = AF_INET;
	server_address.sin_port   = htons(SERVER_PORT_1);
	retVal = inet_aton(SERVER_ADDRESS, &server_address.sin_addr);
	CHECK_RETURN(retVal, "inet_aton");      // NB: There was originally a close(client_socket) in this if.
    
    printf("done.\n");
    
    // Connect to the server.
    printf("Connecting to server... ");
    fflush(stdout);
    retVal = connect(client_socket, (struct sockaddr*) &server_address, sizeof(server_address));
    CHECK_RETURN(retVal, "connect");
        
    printf("connected to %s:%d.\nInitiating data stream...\n", SERVER_ADDRESS, SERVER_PORT_1);
//    retVal = write(client_socket, buffer, buffer_len);
//    CHECK_RETURN(retVal, "write");

//    retVal = write(client_socket, buffer, buffer_len);
//    CHECK_RETURN(retVal, "write2");

//    close(client_socket);
//    return 0;
    
    
    
    // LOOP HERE
    const int depth_buffer_len = sizeof(XnDepthPixel) * 640 * 480;
    const int image_buffer_len = sizeof(XnRGB24Pixel) * 640 * 480;
    while (!xnOSWasKeyboardHit())
    {
        // Update the data.
//        nRetVal = context.WaitAndUpdateAll();
        nRetVal = context.WaitOneUpdateAll(depth);
        if (nRetVal != XN_STATUS_OK)
        {
            printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
            continue;
        }

        // Read the data into our containers.
        depth.GetMetaData(depthMD);
//        image.GetMetaData(imageMD);
        const XnDepthPixel* depthData = depthMD.Data();
//        const XnRGB24Pixel* imageData = imageMD.RGB24Data();

        // Send the data.
        retVal = write(client_socket, depthData, depth_buffer_len);
        CHECK_RETURN(retVal, "write_depth");
    }
    close(client_socket);

   /* 
    // capture the data
    nRetVal = context.WaitAndUpdateAll();
    while (nRetVal != XN_STATUS_OK)
    {
        printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
        nRetVal = context.WaitAndUpdateAll();
    }

    // read the new data into our containers
    depth.GetMetaData(depthMD);
    image.GetMetaData(imageMD);
    const XnDepthPixel* depthData = depthMD.Data();
    const XnRGB24Pixel* imageData = imageMD.RGB24Data();

    // process (save) the data
    printf("Writing data... ");
    fflush(stdout);
    FILE* f;
    f = fopen("fc_640x480_d.dat", "wb");
    fwrite(depthData, sizeof(XnDepthPixel), depthMD.XRes() * depthMD.YRes(), f);
    fclose(f);
    f = fopen("fc_640x480_c.dat", "wb");
    fwrite(imageData, sizeof(XnRGB24Pixel), imageMD.XRes() * imageMD.YRes(), f);
    fclose(f);
    printf("done\n");
    */
    // TO HERE
    
    

    // finish up
    depth.Release();
    image.Release();
    scriptNode.Release();
    context.Release();

    return 0;
}
