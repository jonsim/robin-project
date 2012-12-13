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

	const char *fn = NULL;
	if	(fileExists(SAMPLE_XML_PATH)) fn = SAMPLE_XML_PATH;
	else if (fileExists(SAMPLE_XML_PATH_LOCAL)) fn = SAMPLE_XML_PATH_LOCAL;
	else {
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
	else if (nRetVal != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		return (nRetVal);
	}

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
//    bool imageCaptured = false;
    int frameNumber = 0;

	// main loop
    while (!xnOSWasKeyboardHit())
	{
        // update the data
//        nRetVal = context.WaitAnyUpdateAll();
		nRetVal = context.WaitOneUpdateAll(depth);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
			continue;
		}

        // read the new data into our containers
		xnFPSMarkFrame(&xnFPS);
		depth.GetMetaData(depthMD);
        image.GetMetaData(imageMD);

        // pass this down into our variables
        frameNumber = depthMD.FrameID();
        const XnDepthPixel* depthData = depthMD.Data();
//        const XnUInt8*      imageData = imageMD.Data();
        const XnGrayscale8Pixel* imageData = imageMD.Grayscale8Data();

        // process the data
        printf("Frame %d) %dx%d @ %.1f FPS\n", frameNumber, depthMD.XRes(), depthMD.YRes(), xnFPSCalc(&xnFPS));

        if (frameNumber == 50)
        {
            FILE* f;
//            f = fopen("fc_640x480_d.csv","w");
            f = fopen("fc_640x480_d.dat", "wb");
//            char fileName[64];
//            sprintf(fileName, "fc_%dx%d_%c.dat", depthMD.XRes(), depthMD.YRes(), 'd');
//            f = fopen(fileName,"wb");
//            fprintf(f, "d 640 480\n");
//            printf("sizeof(XnDepthPixel)=%d\n", sizeof(XnDepthPixel));
            fwrite(depthData, sizeof(XnDepthPixel), depthMD.XRes() * depthMD.YRes(), f);
/*            for (XnUInt y = 0; y < depthMD.YRes(); y++)
            {
                for (XnUInt x = 0; x < depthMD.XRes(); x++, depthData++, imageData++)
                {
                    fprintf(f, "%d ", *depthData);
                }
                fprintf(f, "\n");
            }*/
            fclose(f);
        }
//		printf("Frame %d) Middle point is: %u. FPS: %f\n", depthMD.FrameID(), depthMD(depthMD.XRes() / 2, depthMD.YRes() / 2), xnFPSCalc(&xnFPS));
	}

	depth.Release();
    image.Release();
	scriptNode.Release();
	context.Release();

	return 0;
}
