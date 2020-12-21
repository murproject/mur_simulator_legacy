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

#include "zmq.h"
#include <chrono>
#include <cstdlib>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/RenderSurface.h>
#include <Urho3D/Graphics/Skybox.h>
#include <Urho3D/Graphics/Terrain.h>
#include <Urho3D/Graphics/Texture2D.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/UI/ListView.h>
#include <Urho3D/UI/Button.h>
#include <Urho3D/UI/ScrollView.h>

#include <Urho3D/UI/CheckBox.h>
#include <Urho3D/UI/LineEdit.h>
#include <Urho3D/UI/Slider.h>
#include <Urho3D/UI/ToolTip.h>
#include <Urho3D/UI/UIEvents.h>
#include <Urho3D/UI/Window.h>
#include <Urho3D/UI/DropDownList.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/Constraint.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/RenderPath.h>

#include "Water.h"
#include <opencv2/opencv.hpp>
#include <Urho3D/DebugNew.h>

URHO3D_DEFINE_APPLICATION_MAIN(Water)


long long int MyTimer::elapsed() {
    return m_isStarted ?
           std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - m_start).count()
                       : 0;
}

void MyTimer::start() {
    m_start = std::chrono::steady_clock::now();
    m_isStarted = true;
}

void MyTimer::stop() {
    m_isStarted = false;
}

bool MyTimer::isStarted() {
    return m_isStarted;
}

Water::Water(Context *context) :
        Sample(context) {
    m_uiRoot  = GetSubsystem<UI>()->GetRoot();
    MurRobot::RegisterObject(context);
    m_zmqContext = zmq_ctx_new();
    int hwmOption = 1;

    m_zmqBottomData = zmq_socket(m_zmqContext, ZMQ_PUB);
    zmq_setsockopt(m_zmqBottomData, ZMQ_SNDHWM, &hwmOption, sizeof(int));
    zmq_bind(m_zmqBottomData, "tcp://127.0.0.1:1771");

    m_zmqFrontData = zmq_socket(m_zmqContext, ZMQ_PUB);
    zmq_setsockopt(m_zmqFrontData, ZMQ_SNDHWM, &hwmOption, sizeof(int));
    zmq_bind(m_zmqFrontData, "tcp://127.0.0.1:1772");

    m_zmqTelimetryData = zmq_socket(m_zmqContext, ZMQ_PUB);
    zmq_setsockopt(m_zmqTelimetryData, ZMQ_SNDHWM, &hwmOption, sizeof(int));
    zmq_bind(m_zmqTelimetryData, "tcp://127.0.0.1:3390");

    m_userApiPair = zmq_socket(m_zmqContext, ZMQ_PAIR);
    zmq_bind(m_userApiPair, "tcp://127.0.0.1:3391");

    m_thrustersUpdateThread = std::thread([this] {
        while (m_isUpdating) {
            zmq_msg_t thrusters_data{};
            zmq_msg_init(&thrusters_data);
            if (zmq_msg_recv(&thrusters_data, m_userApiPair, 0) != -1) {
                std::memcpy(&m_thrustersData, zmq_msg_data(&thrusters_data), zmq_msg_size(&thrusters_data));
                m_isThrustersData = true;
                if (!m_startTimer.isStarted()) {
                    m_startTimer.start();
                }
            } else {
                m_isThrustersData = false;
                m_startTimer.stop();
            }
            zmq_msg_close(&thrusters_data);
        }
    });
    m_thrustersUpdateThread.detach();

    std::srand(static_cast<unsigned int>(time(nullptr)));
    m_defaultScene = "Scenes/MurMain.xml";

}

void Water::Start() {
    Sample::Start();
    CreateScene();
    CreateVehicle();
    SetupViewport();
    CreateInstructions();
    //CreateVehicle();
    CreateSimulatorWindow();
    CreateSimulatorControls();
    SubscribeToEvents();
    Sample::InitMouseMode(MM_RELATIVE);

    Input* input = GetSubsystem<Input>();
    input->SetMouseMode(MM_FREE);
    input->SetMouseVisible(true);
    m_window->SetVisible(false);
}

void Water::CreateVehicle() {
    scene_->GetComponent<PhysicsWorld>()->SetGravity(Vector3::DOWN * -0.3f);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto vehicleNode = scene_->CreateChild("MurRobot");
    vehicleNode->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
    vehicle_ = vehicleNode->CreateComponent<MurRobot>();
    vehicle_->Init();
}

void Water::CreateScene() {
    auto cache = GetSubsystem<ResourceCache>();

    scene_ = new Scene(context_);

    auto sceneFile = cache->GetResource<XMLFile>(m_defaultScene);
    scene_->LoadXML(sceneFile->GetRoot());
    XMLFile* style = cache->GetResource<XMLFile>("UI/DefaultStyle.xml");
    m_uiRoot->SetDefaultStyle(style);


    waterNode_ = scene_->CreateChild("Water");
    waterNode_->SetScale(Vector3(2048.0f, 0.001f, 2048.0f));
    waterNode_->SetPosition(Vector3(0.0f, 10.0f, 0.0f));

    auto water = waterNode_->CreateComponent<StaticModel>();
    water->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    water->SetMaterial(cache->GetResource<Material>("Materials/Water.xml"));

    auto waterNode1_ = scene_->CreateChild("Water");
    waterNode1_->SetScale(Vector3(2048.0f, 0.001f, 2048.0f));
    waterNode1_->SetPosition(Vector3(0.0f,-7.14296f, 0.0f));

    auto water1 = waterNode1_->CreateComponent<StaticModel>();
    water1->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    water1->SetMaterial(cache->GetResource<Material>("Materials/Water.xml"));


}

void Water::CreateInstructions() {
    auto cache = GetSubsystem<ResourceCache>();
    auto ui = GetSubsystem<UI>();
    text = ui->GetRoot()->CreateChild<Text>();
    text->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    text->SetTextAlignment(HA_LEFT);
    text->SetHorizontalAlignment(HA_LEFT);
    text->SetVerticalAlignment(VA_BOTTOM);
    text->SetPosition(0, 0);

    m_timer = ui->GetRoot()->CreateChild<Text>();
    m_timer->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    m_timer->SetTextAlignment(HA_LEFT);
    m_timer->SetHorizontalAlignment(HA_LEFT);
    m_timer->SetVerticalAlignment(VA_TOP);
    m_timer->SetPosition(0, 0);
    m_timer->SetText("2:0");
}

void Water::SetupViewport() {
    auto renderer = GetSubsystem<Renderer>();
    auto graphics = GetSubsystem<Graphics>();

    renderer->SetHDRRendering(true);
    renderer->SetNumViewports(3);

    cameraNode_ = new Node(context_);
    m_frontCamera = new Node(context_);
    m_downCamera = new Node(context_);
    m_miniDownCamera = m_downCamera->CreateChild("MiniDownCamera");
    m_miniFrontCamera = m_frontCamera->CreateChild("MiniFrontCamera");

    Camera* mdc = m_miniDownCamera->CreateComponent<Camera>();
    Camera* mfc = m_miniFrontCamera->CreateComponent<Camera>();
    mdc->SetFarClip(300.0f);
    mfc->SetFarClip(300.0f);

    auto camera = cameraNode_->CreateComponent<Camera>();
    auto frontCamera = m_frontCamera->CreateComponent<Camera>();
    auto downCamera = m_downCamera->CreateComponent<Camera>();

    downCamera->SetFarClip(500.0f);
    frontCamera->SetFarClip(500.0f);
    camera->SetFarClip(500.0f);
    m_downCamera->Rotate(Quaternion(90.0f, Vector3::RIGHT));

    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    SharedPtr<Viewport> downViewport(new Viewport(context_, scene_, m_downCamera->GetComponent<Camera>()));
    SharedPtr<Viewport> frontViewport(new Viewport(context_, scene_, m_frontCamera->GetComponent<Camera>()));
    SharedPtr<Viewport> miniFrontViewport(new Viewport(context_, scene_, m_miniFrontCamera->GetComponent<Camera>(),
                                                  IntRect(graphics->GetWidth() * 2 / 3, 32, graphics->GetWidth() - 32, graphics->GetHeight() / 3)));
    SharedPtr<Viewport> miniDownViewport(new Viewport(context_, scene_, m_miniDownCamera->GetComponent<Camera>(),
                                                       IntRect(graphics->GetWidth() * 2 / 3, 32 + 256, graphics->GetWidth() - 32, graphics->GetHeight() / 3 + 256)));

    renderer->SetViewport(1, miniFrontViewport);
    renderer->SetViewport(2, miniDownViewport);




    GetSubsystem<Renderer>()->SetViewport(0, viewport);
    m_renderTextureFront = new Texture2D(context_);
    m_renderTextureBottom = new Texture2D(context_);

    m_renderTextureFront->SetSize(320, 240, Graphics::GetRGBFormat(), Urho3D::TEXTURE_RENDERTARGET);
    m_renderTextureBottom->SetSize(320, 240, Graphics::GetRGBFormat(), Urho3D::TEXTURE_RENDERTARGET);

    m_renderTextureFront->SetFilterMode(Urho3D::FILTER_NEAREST);
    m_renderTextureBottom->SetFilterMode(Urho3D::FILTER_NEAREST);

    RenderSurface *renderSurfaceBottom = m_renderTextureBottom->GetRenderSurface();
    renderSurfaceBottom->SetViewport(0, downViewport);
    renderSurfaceBottom->SetUpdateMode(SURFACE_UPDATEALWAYS);
    renderSurfaceBottom->QueueUpdate();

    RenderSurface *renderSurfaceFront = m_renderTextureFront->GetRenderSurface();
    renderSurfaceFront->SetViewport(0, frontViewport);
    renderSurfaceFront->SetUpdateMode(SURFACE_UPDATEALWAYS);
    renderSurfaceFront->QueueUpdate();

}

void Water::SubscribeToEvents() {
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(Water, HandleUpdate));
    SubscribeToEvent(E_POSTUPDATE, URHO3D_HANDLER(Water, HandlePostUpdate));
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(Water, HandlePostRenderUpdate));

}

void Water::MoveCamera(float timeStep) {

}

int checkOver(int p) {
    if (p > 100) p = 100;
    if (p < -100) p = -100;
    return p;
}

void Water::HandleUpdate(StringHash eventType, VariantMap &eventData) {
    using namespace Update;

    static int randA = std::rand() % 10;
    static int randB = std::rand() % 10;
    static int randC = std::rand() % 10;

    Input *input = GetSubsystem<Input>();

    if (m_isThrustersData) {

        int powerA = m_thrustersData.thruster_1 + randA;
        if (m_thrustersData.thruster_1 == 0) {
            powerA = 0;
        }

        int powerB = m_thrustersData.thruster_2 + randB;
        if (m_thrustersData.thruster_2 == 0) {
            powerB = 0;
        }

        int powerC = m_thrustersData.thruster_3 + randC;
        if (m_thrustersData.thruster_3 == 0) {
            powerC = 0;
        }
        vehicle_->SetThrusterPower(checkOver(powerA) , checkOver(powerB),
                                   checkOver(powerC));
    }


    if (vehicle_) {
        UI *ui = GetSubsystem<UI>();

        // Get movement controls and assign them to the vehicle component. If UI has a focused element, clear controls
        if (!ui->GetFocusElement()) {
            int rSteering = 0.0f;
            int lSteering = 0.0f;
            int accelerator = 0.0f;
            int frwd_accelerator = 0.0f;
            // Read controls
            Input *input = GetSubsystem<Input>();
            if (input->GetKeyDown(KEY_A))
                lSteering = 55;
            if (input->GetKeyDown(KEY_D))
                rSteering = 55;
            if (input->GetKeyDown(KEY_UP))
//                accelerator += 1;
                accelerator = 100;
            if (input->GetKeyDown(KEY_DOWN))
//                accelerator -= 1;
                accelerator = -100;
            if (input->GetKeyDown(KEY_W))
//                frwd_accelerator += 1;
                frwd_accelerator = 100;
            if (input->GetKeyDown(KEY_S))
//                frwd_accelerator-=1;
                frwd_accelerator = -100;
            if (!m_isThrustersData) {
                vehicle_->SetThrusterPower(checkOver(frwd_accelerator - lSteering), checkOver(frwd_accelerator - rSteering),
                                           checkOver(accelerator));
            }
            float depth = (200.0f * ((8.2f - vehicle_->GetPosition().y_) / 14.2f));
            //Add yaw & pitch from the mouse motion or touch input. Used only for the camera, does not affect motion
            text->SetText("Roll: " + String((int) vehicle_->GetRoll()) + "\n" +
                          "Pitch: " + String((int) vehicle_->GetPitch()) + "\n" +
                          "Yaw: " + String((int) vehicle_->GetYaw()) + "\n" +
                          "Depth: " + String(depth) + "\n" +
                          "Press F7 to reset simulator state");
//                          "power: A - " + String(frwd_accelerator - lSteering) +
//                          " B - " + String(frwd_accelerator - rSteering) +
//                          " C - " + String(accelerator));
            if (touchEnabled_) {
                for (unsigned i = 0; i < input->GetNumTouches(); ++i) {
                    TouchState *state = input->GetTouch(i);
                    if (!state->touchedElement_)    // Touch on empty space
                    {
                        auto camera = cameraNode_->GetComponent<Camera>();
                        if (!camera)
                            return;

                        auto graphics = GetSubsystem<Graphics>();
                        vehicle_->controls_.yaw_ +=
                                TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.x_;
                        vehicle_->controls_.pitch_ +=
                                TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.y_;
                    }
                }
            } else {
                vehicle_->controls_.yaw_ += (float) input->GetMouseMoveX() * YAW_SENSITIVITY;
                vehicle_->controls_.pitch_ += (float) input->GetMouseMoveY() * YAW_SENSITIVITY;
            }
            // Limit pitch
            if ((vehicle_->GetPosition().y_ + 1.8f) > 10.0f) {
                vehicle_->WaterForce(Vector3::DOWN * 50);
            }

            vehicle_->controls_.pitch_ = Clamp(vehicle_->controls_.pitch_, 0.0f, 80.0f);

            // Check for loading / saving the scene
            if (input->GetKeyPress(KEY_F5)) {
                m_window->SetVisible(!m_window->IsVisible());
            }
            if (input->GetKeyPress(KEY_F7)) {
                auto vehicleNode = scene_->GetChild("MurRobot");
                vehicleNode->SetPosition(Vector3(0, 5.0f, 0));
                vehicleNode->SetRotation(Quaternion(Vector3(0.0f, 0.0f, 0.0f), Vector3::UP));
                m_thrustersData = ThrustersData{};
                cameraNode_->SetPosition(Vector3(0, 5.0f, 0));
                m_isThrustersData = false;
                m_startTimer.stop();
                m_timer->SetColor(Color(255, 255, 255));
            }

        } else
            vehicle_->controls_.Set(CTRL_FORWARD | CTRL_BACK | CTRL_LEFT | CTRL_RIGHT, false);
    }

    if (m_startTimer.isStarted()) {
        long long int time = (60 * 2) - m_startTimer.elapsed();
        long long int seconds = time % 60;
        long long int minute = time / 60;
        if (time <= 0) {
            m_timer->SetText("0:0");
        } else {
            m_timer->SetText(String(minute) + ":" + String(seconds));
        }
        if (m_startTimer.elapsed() >= (60 * 2)) {
            m_timer->SetColor(Color(255, 0 ,0));
        }
    } else {
        m_timer->SetText("2:0");
    }


}


void Water::HandlePostUpdate(StringHash eventType, VariantMap &eventData) {
    if (!vehicle_) {
        return;
    }

    Node *vehicleNode = vehicle_->GetNode();

    Quaternion dir(vehicleNode->GetRotation().YawAngle(), Vector3::UP);
    dir = dir * Quaternion(vehicle_->controls_.yaw_, Vector3::UP);
    dir = dir * Quaternion(vehicle_->controls_.pitch_, Vector3::RIGHT);

    Vector3 cameraTargetPos = vehicleNode->GetPosition() - dir * Vector3(0.0f, 0.0f, 10.0f);
    const Vector3 &cameraStartPos = vehicleNode->GetPosition();

    Ray cameraRay(cameraStartPos, cameraTargetPos - cameraStartPos);
    float cameraRayLength = (cameraTargetPos - cameraStartPos).Length();
    PhysicsRaycastResult result;
    scene_->GetComponent<PhysicsWorld>()->RaycastSingle(result, cameraRay, cameraRayLength, 2);
    if (result.body_) {
        cameraTargetPos = cameraStartPos + cameraRay.direction_ * (result.distance_ - 0.5f);
    }

    cameraNode_->SetPosition(cameraTargetPos);
    cameraNode_->SetRotation(dir);

    m_downCamera->SetPosition(vehicleNode->GetPosition());
    Quaternion rot = m_downCamera->GetRotation();
    rot.FromEulerAngles(90, vehicleNode->GetRotation().YawAngle(), 0);
    m_downCamera->SetRotation(rot);

    Vector3 frontPos = vehicleNode->GetPosition();
    frontPos += Vector3(0, 2.8, 0);
    m_frontCamera->SetPosition(frontPos);

    Quaternion frontRot = m_downCamera->GetRotation();
    frontRot.FromEulerAngles(0, vehicleNode->GetRotation().YawAngle(), 0);
    m_frontCamera->SetRotation(frontRot);
}

void Water::HandlePostRenderUpdate(StringHash eventType, VariantMap &eventData) {
    //scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);
    static std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    size_t count = static_cast<size_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count());
    if (count < 30LL) {
        return;
    }
    std::vector<uchar> buf;
    void *buffer = new char[320 * 240 * 3];
    m_renderTextureBottom->GetData(0, buffer);
    cv::Mat image(240, 320, CV_8UC3, buffer);
    cv::cvtColor(image, image, CV_RGB2BGR);
    cv::imencode(".jpg", image, buf);

    zmq_msg_t bottomImage{};
    zmq_msg_init_size(&bottomImage, buf.size());
    std::memcpy(zmq_msg_data(&bottomImage), buf.data(), buf.size());

    if (-1 == zmq_msg_send(&bottomImage, m_zmqBottomData, 0)) {
        zmq_msg_close(&bottomImage);
        return;
    }
    zmq_msg_close(&bottomImage);
    delete[] buffer;

    buffer = new char[320 * 240 * 3];
    buf.clear();
    image.release();
    m_renderTextureFront->GetData(0, buffer);
    image = cv::Mat(240, 320, CV_8UC3, buffer);
    cv::cvtColor(image, image, CV_RGB2BGR);

    cv::imencode(".jpg", image, buf);

    zmq_msg_t frontImage{};
    zmq_msg_init_size(&frontImage, buf.size());
    std::memcpy(zmq_msg_data(&frontImage), buf.data(), buf.size());

    if (-1 == zmq_msg_send(&frontImage, m_zmqFrontData, 0)) {
        zmq_msg_close(&frontImage);
        return;
    }
    zmq_msg_close(&frontImage);
    delete[] buffer;


    m_mcuData.data.battery = 94;
    m_mcuData.data.i2c_1_value_1 = (float) (200 * ((8.2 - vehicle_->GetPosition().y_) / 14.2f));
    m_mcuData.data.pitch = vehicle_->GetPitch();
    m_mcuData.data.roll = vehicle_->GetRoll();
    m_mcuData.data.yaw = vehicle_->GetYaw();

    zmq_msg_t mcuData{};
    zmq_msg_init_size(&mcuData, McuData::BINARY_DATA_SIZE);
    std::memcpy(zmq_msg_data(&mcuData), &m_mcuData, McuData::BINARY_DATA_SIZE);

    if (-1 == zmq_msg_send(&mcuData, m_zmqTelimetryData, 0)) {
        zmq_msg_close(&mcuData);
        return;
    }
    zmq_msg_close(&mcuData);


}

void Water::CreateSimulatorWindow() {
    // Create the Window and add it to the UI's root node
    m_window = new Window(context_);
    m_uiRoot->AddChild(m_window);

    // Set Window size and layout settings
    m_window->SetMinWidth(200);
    m_window->SetLayout(LM_VERTICAL, 6, IntRect(6, 6, 6, 6));
    m_window->SetAlignment(HA_CENTER, VA_CENTER);
    m_window->SetName("Window");

    // Create Window 'titlebar' container
    UIElement* titleBar = new UIElement(context_);
    titleBar->SetMinSize(0, 24);
    titleBar->SetVerticalAlignment(VA_TOP);
    titleBar->SetLayoutMode(LM_HORIZONTAL);

    // Create the Window title Text
    Text* windowTitle = new Text(context_);
    windowTitle->SetName("WindowTitle");
    windowTitle->SetText("Setup");

    // Add the controls to the title bar
    titleBar->AddChild(windowTitle);

    // Add the title bar to the Window
    m_window->AddChild(titleBar);

    // Apply styles
    m_window->SetStyleAuto();
    windowTitle->SetStyleAuto();
}

Water::~Water() {
    m_isUpdating = false;
}

void Water::CreateSimulatorControls() {

    FileSystem fs(context_);

    fs.ScanDir(m_files, fs.GetCurrentDir() + "Data\\Scenes", "*.*", SCAN_FILES, false);

    m_slider = new Slider(context_);
    m_slider->SetName("GravitySlider");
    m_slider->SetMinHeight(24);
    m_slider->SetRange(1);
    m_slider->SetValue(0.3);

    m_gravity = new Text(context_);
    m_gravity->SetName("GravityText");
    m_gravity->SetText("Buoyancy: -0.3");
    m_gravity->SetMinHeight(24);


//    m_listView = new ListView(context_);
//    m_listView->SetName("Scenes");
//    m_listView->SetMinHeight(80);
//
//    m_listView->SetScrollBarsAutoVisible(false);

    m_window->AddChild(m_slider);
    m_window->AddChild(m_gravity);
//    m_window->AddChild(m_listView);

    m_slider->SetStyleAuto();
//    m_listView->SetStyleAuto();
    m_gravity->SetStyleAuto();
/*
    for (size_t i = 0; i < m_files.Size(); ++i) {
        Text *t = new Text(context_);
        t->SetText(m_files.At(i));
        t->SetMinHeight(24);
        m_listView->AddItem(t);
        t->SetStyleAuto();
    }
        SubscribeToEvent(m_listView, E_ITEMCLICKED, URHO3D_HANDLER(Water, HandleItemClicked));
*/
    SubscribeToEvent(m_slider, E_SLIDERCHANGED, URHO3D_HANDLER(Water, HandleSliderChanged));

}

void Water::HandleSliderChanged(StringHash eventType, VariantMap &eventData) {
    float val = m_slider->GetValue() * 2.0F - 1.0F;
    m_gravity->SetText("Buoyancy: " + String(val));

    if (val < 0.05F  && val > -0.05F) {
        scene_->GetComponent<PhysicsWorld>()->SetGravity(Vector3::ZERO);
    } else {
        scene_->GetComponent<PhysicsWorld>()->SetGravity(Vector3::DOWN * val);
    }

}

void Water::HandleItemClicked(StringHash eventType, VariantMap &eventData) {
    auto cache = GetSubsystem<ResourceCache>();
    m_defaultScene = "Scene/" + m_files.At(m_listView->GetSelection());
}
