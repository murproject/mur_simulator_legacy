//
// Copyright (c) 2008-2017 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#pragma once

#include <Urho3D/Math/Plane.h>
#include <thread>
#include <chrono>
#include "zmq.h"
#include "Sample.h"
#include "MurRobot.h"
namespace Urho3D
{

class Node;
class Scene;
class Window;
class Slider;
class ListView;
class Text;
class DropDownList;
class ScrollView;
}


struct ThrustersData {
    int8_t thruster_1 = 0;
    int8_t thruster_2 = 0;
    int8_t thruster_3 = 0;
    int8_t thruster_4 = 0;
};

#pragma pack (push, 1)
union McuData {
    enum {
        BINARY_DATA_SIZE = 36
    };
    struct {
        uint8_t thruster_1;  //1
        uint8_t thruster_2;  //2
        uint8_t thruster_3;  //3
        uint8_t thruster_4;  //4
        uint8_t button;      //5
        uint8_t i2c_1;       //6
        uint8_t i2c_2;       //7
        uint8_t battery;     //8
        float pitch;         //12
        float roll;          //16
        float yaw;           //20
        float i2c_1_value_1; //24
        float i2c_1_value_2; //28
        float i2c_2_value_1; //32
        float i2c_2_value_2; //36
    } data;

    struct {
        float x;
        float y;
        float z;
    } raw_mag;

    uint8_t binary[BINARY_DATA_SIZE];
};
#pragma pack (pop)

class MyTimer {
public:
    void start();

    void stop();

    long long int elapsed();

    bool isStarted();

private:
    std::chrono::steady_clock::time_point m_start;
    bool m_isStarted = false;
};




class Water : public Sample {
URHO3D_OBJECT(Water, Sample);

public:
    /// Construct.
    Water(Context *context);

    /// Setup after engine initialization and before running the main loop.
    virtual void Start() override;
    ~Water();

private:
    void CreateSimulatorWindow();

    void CreateSimulatorControls();

    void CreateVehicle();

    void CreateScene();

    void CreateInstructions();

    void SetupViewport();

    void SubscribeToEvents();

    void MoveCamera(float timeStep);

    void HandleUpdate(StringHash eventType, VariantMap &eventData);

    void HandlePostUpdate(StringHash eventType, VariantMap &eventData);

    void HandlePostRenderUpdate(StringHash eventType, VariantMap &eventData);

    void HandleSliderChanged(StringHash eventType, VariantMap &eventData);

    void HandleItemClicked(StringHash eventType, VariantMap &eventData);

    SharedPtr<Node> reflectionCameraNode_;
    SharedPtr<Node> waterNode_;
    SharedPtr<Node> m_frontCamera;
    SharedPtr<Node> m_downCamera;
    SharedPtr<Node> m_miniDownCamera;
    SharedPtr<Node> m_miniFrontCamera;

    Plane waterPlane_;
    Plane waterClipPlane_;
    Text *text;
    WeakPtr<MurRobot> vehicle_;

    SharedPtr<Texture2D> m_renderTextureFront;
    SharedPtr<Texture2D> m_renderTextureBottom;

    SharedPtr<Window> m_window;
    SharedPtr<UIElement> m_uiRoot;


    void *m_zmqContext;
    void *m_zmqBottomData;
    void *m_zmqTelimetryData;
    void *m_zmqFrontData;
    void *m_userApiPair;
    ThrustersData m_thrustersData;
    McuData m_mcuData;
    bool m_isUpdating = true;
    bool m_isThrustersData = false;
    std::thread m_thrustersUpdateThread;
    Slider *m_slider;
    ListView *m_listView;
    Text *m_gravity;
    Text *m_timer;
    MyTimer m_startTimer;
    Vector<String> m_files;
    String m_defaultScene;

};

