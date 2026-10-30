// g1_vui_probe — does the G1 run the Go2's "vui" service (lights + volume), and is the light it
// drives the RING around the head? The middle LED is already ours (AudioClient::LedControl,
// /audio/led); nothing in the SDK names the ring, and on the Go2 the head light's brightness
// lives in VuiClient. This asks the G1 that service and dims the light so a person can look.
//
// Build and run ON PC2 (never part of the relay; a probe):
//   g++ -O2 -std=c++17 tools/g1_vui_probe.cpp -o g1_vui_probe \
//       -I ~/unitree_sdk2/include -I ~/unitree_sdk2/thirdparty/include \
//       -I ~/unitree_sdk2/thirdparty/include/ddscxx \
//       ~/unitree_sdk2/lib/aarch64/libunitree_sdk2.a -L ~/unitree_sdk2/thirdparty/lib/aarch64 \
//       -lddscxx -lddsc -lpthread
//   ./g1_vui_probe eth0
//
// It prints every return code: 0 is success; a code in the 31xx range means the service or the
// API is not there (unitree/common/error.hpp), which answers the question too.
#include <chrono>
#include <cstdio>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>

using unitree::robot::ChannelFactory;

static void pause(int s, const char* what) {
    printf("  ... %s: look at the head for %d s\n", what, s);
    fflush(stdout);
    std::this_thread::sleep_for(std::chrono::seconds(s));
}

int main(int argc, char** argv) {
    ChannelFactory::Instance()->Init(0, argc > 1 ? argv[1] : "eth0");
    unitree::robot::go2::VuiClient vui;
    vui.SetTimeout(3.0f);
    vui.Init();

    int sw = -1, bright = -1, vol = -1;
    printf("GetSwitch     rc=%d  value=%d\n", vui.GetSwitch(sw), sw);
    printf("GetBrightness rc=%d  value=%d\n", vui.GetBrightness(bright), bright);
    printf("GetVolume     rc=%d  value=%d\n", vui.GetVolume(vol), vol);
    fflush(stdout);
    if (bright < 0) {
        printf("no brightness read back: the G1 does not answer the vui service this way\n");
        return 1;
    }

    printf("SetBrightness(0)  rc=%d\n", vui.SetBrightness(0));
    pause(4, "brightness 0");
    printf("SetBrightness(10) rc=%d\n", vui.SetBrightness(10));
    pause(4, "brightness 10");
    printf("SetSwitch(0)      rc=%d\n", vui.SetSwitch(0));
    pause(4, "switch off");
    // Leave it as it was.
    printf("SetSwitch(%d)      rc=%d\n", sw < 0 ? 1 : sw, vui.SetSwitch(sw < 0 ? 1 : sw));
    printf("SetBrightness(%d) rc=%d\n", bright, vui.SetBrightness(bright));
    printf("restored\n");
    return 0;
}
