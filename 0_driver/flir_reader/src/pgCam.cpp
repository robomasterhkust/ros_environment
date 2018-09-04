#include "pgCam.hpp"

PointGreyCamera::PointGreyCamera(const string &config_path)
    : Camera(config_path)
{

    pCamera = new FlyCapture2::Camera();
};

PointGreyCamera::~PointGreyCamera()
{
    delete pCamera;
};

bool PointGreyCamera::initialize()
{
    FlyCapture2::Error error;
    if (serialNumber != 0)
    {
        ROS_INFO("Initializing FLIR camera with S/N %d", serialNumber);
        error = busMgr.GetCameraFromSerialNumber(serialNumber, &guid);

        if (error != FlyCapture2::PGRERROR_OK)
        {
            ROS_INFO("ERROR Initializing FLIR camera");
            error.PrintErrorTrace();
            return false;
        }
        else
        {
            ROS_INFO("FLIR camera connected");
        }
    }
    else
    {
        //search for any available camera...
        ROS_INFO("Initializing FLIR camera with any S/N...");
        unsigned int numCameras;
        error = busMgr.GetNumOfCameras(&numCameras);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            ROS_INFO("ERROR finding FLIR camera");
            error.PrintErrorTrace();
            return false;
        }
        else
        {
            ROS_INFO("Found %d FLIR camera", numCameras);
        }

        if (numCameras > 0)
        {
            error = busMgr.GetCameraFromIndex(0, &guid);
            if (error != FlyCapture2::PGRERROR_OK)
            {
                error.PrintErrorTrace();
                return false;
            }
        }
    }

    //connect to camera
    error = pCamera->Connect(&guid);

    if (error != FlyCapture2::PGRERROR_OK)
    {
        std::cout << "[#INFO] Error in Connect." << std::endl;
        error.PrintErrorTrace();
    }
    else
    {
        if (pCamera->IsConnected())
        {
            std::cout << "[#INFO] Camera Connected." << std::endl;
        }
    }

    FlyCapture2::CameraInfo camInfo;
    error = pCamera->GetCameraInfo(&camInfo);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return false;
    }
    realserialNumber = camInfo.serialNumber;

    printf("Started FLIR camera:");
    printf("Serial number: %d\n", realserialNumber);
    printf("Camera model: %s\n", camInfo.modelName);

    cameraConfig.numBuffers = 10;
    cameraConfig.grabTimeout = FlyCapture2::TIMEOUT_UNSPECIFIED;
    cameraConfig.grabMode = FlyCapture2::DROP_FRAMES;

    error = pCamera->SetConfiguration(&cameraConfig);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        std::cout << "[#INFO]Error in setCameraConfiguration " << std::endl;
        error.PrintErrorTrace();
        return false;
    }

    FlyCapture2::EmbeddedImageInfo info;
    info.timestamp.onOff = true;
    info.gain.onOff = true;
    info.shutter.onOff = true;
    info.brightness.onOff = true;
    info.exposure.onOff = true;
    info.whiteBalance.onOff = true;
    info.frameCounter.onOff = true;
    info.ROIPosition.onOff = true;
    error = pCamera->SetEmbeddedImageInfo(&info);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        std::cout << "[#INFO]Error in setMetadata." << std::endl;
        error.PrintErrorTrace();
        return false;
    }

    return true;
};

void PointGreyCamera::discardFrame(){
    //not implemented yet as since not needed
};

FrameInfo *PointGreyCamera::getFrame()
{
    FrameInfo *tempOut = new FrameInfo(this);

    FlyCapture2::Image rawImage;

    // Retrieve an image
    FlyCapture2::Error error = pCamera->RetrieveBuffer(&rawImage);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        ROS_INFO("Error in RetrieveBuffer, captureOneImage");
        return NULL;
    }

    FlyCapture2::TimeStamp ts = rawImage.GetTimeStamp();
    //    std::cout << "time " << time.seconds << " " << time.microSeconds << std::endl;

    // Create a converted image
    FlyCapture2::Image convertedImage;

    // Convert the raw image
    error = rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &convertedImage);

    if (error != FlyCapture2::PGRERROR_OK)
    {
        ROS_INFO("Error in Convert");
        error.PrintErrorTrace();
        return NULL;
    }

    // Change to opencv image Mat
    unsigned char *pdata = convertedImage.GetData();

    Mat tempimg = cv::Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC3, pdata);
    if (tempimg.empty())
        return NULL;

    if (resizeHalf)
    {
        cv::resize(tempimg, tempOut->img, cv::Size(tempimg.cols / 2, tempimg.rows / 2), 0, 0);
    }
    else
    {
        tempimg.copyTo(tempOut->img);
    }

    tempOut->rosheader.stamp.nsec = ts.microSeconds * 1000;
    tempOut->rosheader.stamp.sec = ts.seconds;

    return tempOut;
};

bool PointGreyCamera::loadDriverParameters(const FileStorage &fs)
{

    const FileNode &node = fs["pgCam"];
    fsHelper::readOrDefault(node["serialNumber"], (int &)serialNumber, 0);

    fsHelper::readOrDefault(node["resizeHalf"], resizeHalf, false);
    fsHelper::readOrDefault(node["useTrigger"], useTrigger, false);

    fsHelper::readOrDefault(node["format7_mode"], (int &)format7ImageSettings.mode, (int)FlyCapture2::MODE_0);

    fsHelper::readOrDefault(node["frameRate_use_abs"], frameRate_use_abs, true);
    fsHelper::readOrDefault(node["frameRate_use_auto"], frameRate_use_auto, false);
    fsHelper::readOrDefault(node["frameRate_abs"], frameRate_abs, 50.0f);
    fsHelper::readOrDefault(node["frameRate"], (int &)frameRate, 50);

    fsHelper::readOrDefault(node["brightness_use_abs"], brightness_use_abs, true);
    fsHelper::readOrDefault(node["brightness_use_auto"], brightness_use_auto, false);
    fsHelper::readOrDefault(node["brightness"], (int &)brightness, 10);
    fsHelper::readOrDefault(node["brightness_abs"], brightness_abs, 1.0f);

    fsHelper::readOrDefault(node["exposure_use_abs"], exposure_use_abs, true);
    fsHelper::readOrDefault(node["exposure_use_auto"], exposure_use_auto, false);
    fsHelper::readOrDefault(node["exposure_abs"], exposure_abs, 10.0f);
    fsHelper::readOrDefault(node["exposure"], (int &)exposure, 10);

    fsHelper::readOrDefault(node["saturation_use_abs"], saturation_use_abs, true);
    fsHelper::readOrDefault(node["saturation_abs"], saturation_abs, 50.0f);
    fsHelper::readOrDefault(node["saturation"], (int &)saturation, 512);

    fsHelper::readOrDefault(node["hue_use_abs"], hue_use_abs, true);
    fsHelper::readOrDefault(node["hue_abs"], hue_abs, 0.0f);
    fsHelper::readOrDefault(node["hue"], (int &)hue, 2046);

    fsHelper::readOrDefault(node["sharpness_use_abs"], sharpness_use_abs, false);
    fsHelper::readOrDefault(node["sharpness_abs"], sharpness_abs, 0.0f);
    fsHelper::readOrDefault(node["sharpness"], (int &)sharpness, 0);

    fsHelper::readOrDefault(node["WB_use_auto"], WB_use_auto, false);
    fsHelper::readOrDefault(node["WB_red"], (int &)WB_red, 550);
    fsHelper::readOrDefault(node["WB_blue"], (int &)WB_blue, 810);

    fsHelper::readOrDefault(node["gamma_use_abs"], gamma_use_abs, true);
    fsHelper::readOrDefault(node["gamma_use_auto"], gamma_use_auto, false);
    fsHelper::readOrDefault(node["gamma"], (int &)gamma, 1);
    fsHelper::readOrDefault(node["gamma_abs"], gamma_abs, 1.0f);

    fsHelper::readOrDefault(node["shutter_use_abs"], shutter_use_abs, true);
    fsHelper::readOrDefault(node["shutter_use_auto"], shutter_use_auto, false);
    fsHelper::readOrDefault(node["shutter"], (int &)shutter, 10);
    fsHelper::readOrDefault(node["shutter_abs"], shutter_abs, 2.0f);

    fsHelper::readOrDefault(node["gain_use_abs"], gain_use_abs, true);
    fsHelper::readOrDefault(node["gain_use_auto"], gain_use_auto, false);
    fsHelper::readOrDefault(node["gain"], (int &)gain, 10);
    fsHelper::readOrDefault(node["gain_abs"], gain_abs, 0.5f);

    return true;
};

bool PointGreyCamera::storeDriverParameters(FileStorage &fs)
{
    fs << "pgCam"
       << "{"
       << "serialNumber" << (int)serialNumber

       << "resizeHalf" << resizeHalf
       << "useTrigger" << useTrigger

       << "format7_mode" << format7ImageSettings.mode

       << "brightness_use_abs" << brightness_use_abs
       << "brightness_use_auto" << brightness_use_auto
       << "brightness_abs" << brightness_abs
       << "brightness" << (int)brightness

       << "frameRate_use_abs" << frameRate_use_abs
       << "frameRate_use_auto" << frameRate_use_auto
       << "frameRate_abs" << frameRate_abs
       << "frameRate" << (int)frameRate

       << "exposure_use_abs" << exposure_use_abs
       << "exposure_use_auto" << exposure_use_auto
       << "exposure_abs" << exposure_abs
       << "exposure" << (int)exposure

       << "saturation_use_abs" << saturation_use_abs
       << "saturation_abs" << saturation_abs
       << "saturation" << (int)saturation

       << "hue_use_abs" << hue_use_abs
       << "hue_abs" << hue_abs
       << "hue" << (int)hue

       << "sharpness_use_abs" << sharpness_use_abs
       << "sharpness_abs" << sharpness_abs
       << "sharpness" << (int)sharpness

       << "WB_use_auto" << WB_use_auto
       << "WB_red" << (int)WB_red
       << "WB_blue" << (int)WB_blue

       << "gamma_use_abs" << gamma_use_abs
       << "gamma_use_auto" << gamma_use_auto
       << "gamma_abs" << gamma_abs
       << "gamma" << (int)gamma

       << "shutter_use_abs" << shutter_use_abs
       << "shutter_use_auto" << shutter_use_auto
       << "shutter_abs" << shutter_abs
       << "shutter" << (int)shutter

       << "gain_use_abs" << gain_use_abs
       << "gain_use_auto" << gain_use_auto
       << "gain_abs" << gain_abs
       << "gain" << (int)gain
       << "}";
};

bool PointGreyCamera::setCamConfig()
{
    FlyCapture2::Error error;

    setCamProperty("Frame rate",
                   FlyCapture2::FRAME_RATE,
                   frameRate_use_auto,
                   frameRate_use_abs,
                   frameRate_abs,
                   frameRate,
                   0);

    setCamProperty("Brightness",
                   FlyCapture2::BRIGHTNESS,
                   brightness_use_auto,
                   brightness_use_abs,
                   brightness_abs,
                   brightness,
                   0);

    setCamProperty("Exposure",
                   FlyCapture2::AUTO_EXPOSURE,
                   exposure_use_auto,
                   exposure_use_abs,
                   exposure_abs,
                   exposure,
                   0);

    setCamProperty("Hue",
                   FlyCapture2::HUE,
                   false,
                   hue_use_abs,
                   hue_abs,
                   hue,
                   0);

    setCamProperty("Saturation",
                   FlyCapture2::SATURATION,
                   false,
                   saturation_use_abs,
                   saturation_abs,
                   saturation,
                   0);

    setCamProperty("Sharpness",
                   FlyCapture2::SHARPNESS,
                   false,
                   sharpness_use_abs,
                   sharpness_abs,
                   sharpness,
                   0);

    setCamProperty("White balance",
                   FlyCapture2::WHITE_BALANCE,
                   WB_use_auto,
                   false,
                   0,
                   WB_red,
                   WB_blue);

    setCamProperty("Gamma",
                   FlyCapture2::GAMMA,
                   gamma_use_auto,
                   gamma_use_abs,
                   gamma_abs,
                   gamma,
                   0);

    setCamProperty("Shutter speed",
                   FlyCapture2::SHUTTER,
                   shutter_use_auto,
                   shutter_use_abs,
                   shutter_abs,
                   shutter,
                   0);

    setCamProperty("Gain",
                   FlyCapture2::GAIN,
                   gain_use_auto,
                   gain_use_abs,
                   gain_abs,
                   gain,
                   0);

    FlyCapture2::TriggerMode tm;
    tm.mode = 0; // 0 means stand trigger
    tm.onOff = useTrigger;
    tm.polarity = true;

    error = pCamera->SetTriggerMode(&tm);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        ROS_INFO("Error setting trigger mode: %s", error.GetDescription());
    }

    return true;
};

bool PointGreyCamera::getCamConfig()
{

    getCamProperty("Brightness",
                   FlyCapture2::BRIGHTNESS,
                   &brightness_use_auto,
                   &brightness_abs,
                   &brightness,
                   0);

    getCamProperty("Exposure",
                   FlyCapture2::AUTO_EXPOSURE,
                   &exposure_use_auto,
                   &exposure_abs,
                   &exposure,
                   0);

    getCamProperty("Hue",
                   FlyCapture2::HUE,
                   NULL,
                   &hue_abs,
                   &hue,
                   0);

    getCamProperty("Saturation",
                   FlyCapture2::SATURATION,
                   NULL,
                   &saturation_abs,
                   &saturation,
                   0);

    getCamProperty("Sharpness",
                   FlyCapture2::SHARPNESS,
                   NULL,
                   &sharpness_abs,
                   &sharpness,
                   0);

    getCamProperty("White balance",
                   FlyCapture2::WHITE_BALANCE,
                   0,
                   0,
                   &WB_red,
                   &WB_blue);

    getCamProperty("Gamma",
                   FlyCapture2::GAMMA,
                   &gamma_use_auto,
                   &gamma_abs,
                   &gamma,
                   0);

    getCamProperty("Shutter speed",
                   FlyCapture2::SHUTTER,
                   &shutter_use_auto,
                   &shutter_abs,
                   &shutter,
                   0);

    getCamProperty("Gain",
                   FlyCapture2::GAIN,
                   &gain_use_auto,
                   &gain_abs,
                   &gain,
                   0);

    getCamProperty("Frame rate",
                   FlyCapture2::FRAME_RATE,
                   &frameRate_use_auto,
                   &frameRate_abs,
                   &frameRate,
                   0);

    return true;
};

bool PointGreyCamera::startStream()
{
    FlyCapture2::Error error = pCamera->StartCapture();
    return (error == FlyCapture2::PGRERROR_OK);
};

bool PointGreyCamera::closeStream()
{
    FlyCapture2::Error error = pCamera->StopCapture();
    return (error == FlyCapture2::PGRERROR_OK);
};

void PointGreyCamera::info()
{
    showCamProperty("BRIGHTNESS", FlyCapture2::BRIGHTNESS);
    showCamProperty("AUTO_EXPOSURE", FlyCapture2::AUTO_EXPOSURE);
    showCamProperty("SHARPNESS", FlyCapture2::SHARPNESS);
    showCamProperty("WHITE_BALANCE", FlyCapture2::WHITE_BALANCE);
    showCamProperty("HUE", FlyCapture2::HUE);
    showCamProperty("SATURATION", FlyCapture2::SATURATION);
    showCamProperty("GAMMA", FlyCapture2::GAMMA);
    showCamProperty("IRIS", FlyCapture2::IRIS);
    showCamProperty("FOCUS", FlyCapture2::FOCUS);
    showCamProperty("ZOOM", FlyCapture2::ZOOM);
    showCamProperty("PAN", FlyCapture2::PAN);
    showCamProperty("TILT", FlyCapture2::TILT);
    showCamProperty("SHUTTER", FlyCapture2::SHUTTER);
    showCamProperty("GAIN", FlyCapture2::GAIN);
    showCamProperty("TRIGGER_MODE", FlyCapture2::TRIGGER_MODE);
    showCamProperty("TRIGGER_DELAY", FlyCapture2::TRIGGER_DELAY);
    showCamProperty("FRAME_RATE", FlyCapture2::FRAME_RATE);
    showCamProperty("TEMPERATURE", FlyCapture2::TEMPERATURE);
};

bool PointGreyCamera::showCamProperty(const char *name,
                                      FlyCapture2::PropertyType type)
{
    FlyCapture2::PropertyInfo info;
    info.type = type;
    pCamera->GetPropertyInfo(&info);

    printf("Show FLIR Camera Property: \"%s\"\n", name);
    printf("autoSupported: %s\nabsMax: %f\nabsMin: %f\nabsValSupported: %s\nmanualSupported: %s\nmax: %d\nmin: %d\nonePushSupported: %s\nonOffSupported: %s\npresent:  %s\nUnitAbbr: %s\nUnits: %s\nreadOutSupported: %s\n",
           info.autoSupported ? "true" : "false",
           info.absMax,
           info.absMin,
           info.absValSupported ? "true" : "false",
           info.manualSupported ? "true" : "false",
           info.max,
           info.min,
           info.onePushSupported ? "true" : "false",
           info.onOffSupported ? "true" : "false",
           info.present ? "true" : "false",
           info.pUnitAbbr,
           info.pUnits,
           info.readOutSupported ? "true" : "false");
};

bool PointGreyCamera::setCamProperty(const char *name,
                                     FlyCapture2::PropertyType type,
                                     bool use_auto,
                                     bool use_abs,
                                     float value_abs,
                                     unsigned int valueA,
                                     unsigned int valueB)
{
    FlyCapture2::Property prop;
    prop.type = type;
    prop.autoManualMode = use_auto;
    prop.absControl = use_abs;
    prop.absValue = value_abs;
    prop.valueA = valueA;
    prop.valueB = valueB;
    prop.onOff = true;
    FlyCapture2::Error error = pCamera->SetProperty(&prop);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        ROS_INFO("Error setting FLIR Camera Property: \"%s\" : %s", name, error.GetDescription());
        return false;
    }
    else
    {
        ROS_INFO("Set FLIR Cam \"%s\" success", name);
        return true;
    }
};

bool PointGreyCamera::getCamProperty(const char *name,
                                     FlyCapture2::PropertyType type,
                                     bool *use_auto,
                                     float *value_abs,
                                     unsigned int *valueA,
                                     unsigned int *valueB)
{
    FlyCapture2::Property fProp;
    fProp.type = type;
    FlyCapture2::Error error = pCamera->GetProperty(&fProp);

    if (error != FlyCapture2::PGRERROR_OK)
    {
        ROS_INFO("Error getting FLIR Camera Property: \"%s\"\n", name);
        error.PrintErrorTrace();
        return false;
    }
    else if (fProp.present)
    {
        cout << endl;
        ROS_INFO("Got FLIR Camera Property: \"%s\"", name);
        printf(fProp.autoManualMode ? "Auto mode" : "Manual mode");
        printf("\tAbsolute value: %f\n", fProp.absValue);
        printf("\tValue A: %d\tValue B: %d\n", fProp.valueA, fProp.valueB);

        if (use_auto)
            *use_auto = fProp.autoManualMode;
        if (value_abs)
            *value_abs = fProp.absValue;
        if (valueA)
            *valueA = fProp.valueA;
        if (valueB)
            *valueB = fProp.valueB;
        return true;
    }
    else
    {
        ROS_INFO("FLIR Camera Property \"%s\"\n absent", name);
        return false;
    }
};

float PointGreyCamera::getTemperature()
{
    float temp;
    FlyCapture2::Property fProp;
    fProp.type = FlyCapture2::TEMPERATURE;
    FlyCapture2::Error error = pCamera->GetProperty(&fProp);

    if (error != FlyCapture2::PGRERROR_OK)
    {
        ROS_INFO("Error getting FLIR Camera temperature: %s", error.GetDescription());
        return 0;
    }
    else
    {
        ROS_INFO("FLIR Camera %d temperature: %f C", realserialNumber, fProp.absValue * 100.0 - 273.15);
        return fProp.absValue * 100.0 - 273.15;
    }
};

bool PointGreyCamera::solvePNP(cv::InputArray objectPoints, const vector<Point2f> &imagePoints,
                               cv::OutputArray rvec, cv::OutputArray tvec,
                               bool useExtrinsicGuess, int flags) const
{
    vector<Point2f> temp;
    if (resizeHalf)
    {
        for (auto i : imagePoints)
        {
            temp.push_back(i * 2);
        }
        return cv::solvePnP(objectPoints, temp, this->cameraMatrix, this->distCoeffs, rvec, tvec, useExtrinsicGuess, flags);
    }
    return cv::solvePnP(objectPoints, imagePoints, this->cameraMatrix, this->distCoeffs, rvec, tvec, useExtrinsicGuess, flags);
};
