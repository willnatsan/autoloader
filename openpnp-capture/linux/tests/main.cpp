/*

    openpnp test application

    Niels Moseley

*/
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <unistd.h>
#include <chrono>

#include "openpnp-capture.h"
#include "../common/context.h"

int main(int argc, char *argv[])
{
    uint32_t deviceFormatID = 13; // Format 13: 1920x1080; YUYV; 5 FPS
    uint32_t deviceID = 0;        // ID for Top Camera / Bottom Camera

    CapContext ctx = Cap_createContext();

    uint32_t deviceCount = Cap_getDeviceCount(ctx);
    for (uint32_t i = 0; i < deviceCount; i++)
    {
        deviceID = i;
        int32_t streamID = Cap_openStream(ctx, deviceID, deviceFormatID);

        // get current stream parameters
        CapFormatInfo finfo;
        Cap_getFormatInfo(ctx, deviceID, deviceFormatID, &finfo);

        // disable auto exposure and white balance
        Cap_setAutoProperty(ctx, streamID, CAPPROPID_EXPOSURE, 0);
        Cap_setAutoProperty(ctx, streamID, CAPPROPID_WHITEBALANCE, 1);

        int32_t exposure = 0;
        int32_t brightness = -2;
        int32_t bl_compensation = 0;
        int32_t contrast = 64;
        int32_t gamma = 72;
        int32_t hue = 0;
        int32_t pl_freq = 1;
        int32_t saturation = 1;
        int32_t sharpness = 0;
        int32_t wbalance = 4000;

        std::string device_name = Cap_getDeviceName(ctx, deviceID);
        if (device_name == "LumenPnP Top: LumenPnP Top")
        {
            exposure = 146;
        }
        else if (device_name == "LumenPnP Bottom: LumenPnP Botto")
        {
            exposure = 250;
        }

        Cap_setProperty(ctx, streamID, CAPPROPID_EXPOSURE, exposure);
        Cap_setProperty(ctx, streamID, CAPPROPID_BRIGHTNESS, brightness);
        Cap_setProperty(ctx, streamID, CAPPROPID_BACKLIGHTCOMP, bl_compensation);
        Cap_setProperty(ctx, streamID, CAPPROPID_CONTRAST, contrast);
        Cap_setProperty(ctx, streamID, CAPPROPID_GAMMA, gamma);
        Cap_setProperty(ctx, streamID, CAPPROPID_HUE, hue);
        Cap_setProperty(ctx, streamID, CAPPROPID_POWERLINEFREQ, pl_freq);
        Cap_setProperty(ctx, streamID, CAPPROPID_SATURATION, saturation);
        Cap_setProperty(ctx, streamID, CAPPROPID_SHARPNESS, sharpness);

        Cap_closeStream(ctx, streamID);
    }

    CapResult result = Cap_releaseContext(ctx);

    return 0;
}
