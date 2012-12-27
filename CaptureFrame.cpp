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

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
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
	const char *fn = NULL;
	if	(fileExists(SAMPLE_XML_PATH))
        fn = SAMPLE_XML_PATH;
	else if (fileExists(SAMPLE_XML_PATH_LOCAL))
        fn = SAMPLE_XML_PATH_LOCAL;
	else
    {
		printf("Could not find '%s' nor '%s'. Aborting.\n" , SAMPLE_XML_PATH, SAMPLE_XML_PATH_LOCAL);
		return XN_STATUS_ERROR;
	}
	printf("Reading config from: '%s'\n", fn);
	nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);

	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (nRetVal);
	}
    CHECK_RC(nRetVal, "Open");

    // main variables
	DepthGenerator depth;
    ImageGenerator image;
    DepthMetaData  depthMD;
    ImageMetaData  imageMD;
    XnFPSData      xnFPS;

    // initialise and validate main variables
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
	CHECK_RC(nRetVal, "Find depth generator");
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image);
    CHECK_RC(nRetVal, "Find image generator");
	nRetVal = xnFPSInit(&xnFPS, 180);
	CHECK_RC(nRetVal, "FPS Init");

    // some of my cheeky variables
//    int frameNumber = 0;
//    bool imageWritten = false;

	// main loop
//    while (!xnOSWasKeyboardHit())
//	{
        // update the data
//        nRetVal = context.WaitAnyUpdateAll();
//		nRetVal = context.WaitOneUpdateAll(depth);
        nRetVal = context.WaitAndUpdateAll();

        while (nRetVal != XN_STATUS_OK)
		{
			printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
            nRetVal = context.WaitAndUpdateAll();
		}

        // read the new data into our containers
//		xnFPSMarkFrame(&xnFPS);
		depth.GetMetaData(depthMD);
        image.GetMetaData(imageMD);

        // pass this down into our variables
//        frameNumber = depthMD.FrameID();
        const XnDepthPixel* depthData = depthMD.Data();
        const XnRGB24Pixel* imageData = imageMD.RGB24Data();

        // process the data
//        printf("Frame %d) %.1f FPS\n", frameNumber, xnFPSCalc(&xnFPS));

//        if (frameNumber == 50 && !imageWritten)
//        {
            printf("  Writing data... ");
            fflush(stdout);
            FILE* f;
            f = fopen("fc_640x480_d.dat", "wb");
            fwrite(depthData, sizeof(XnDepthPixel), depthMD.XRes() * depthMD.YRes(), f);
            fclose(f);

            f = fopen("fc_640x480_c.dat", "wb");
            fwrite(imageData, sizeof(XnRGB24Pixel), imageMD.XRes() * imageMD.YRes(), f);
            fclose(f);

//            imageWritten = true;
            printf("done\n");
//        }
//	}

	depth.Release();
    image.Release();
	scriptNode.Release();
	context.Release();

	return 0;
}
