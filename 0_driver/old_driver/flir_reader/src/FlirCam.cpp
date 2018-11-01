#include "FlirCam.hpp"
#include "flycapNames.hpp"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

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
    if (serialNumber == 0)
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
        else
            return false;
    }
    else
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
            ROS_INFO("FLIR camera of serialNumber found");
        }
    }

    //connect to camera
    error = pCamera->Connect(&guid);

    if (error != FlyCapture2::PGRERROR_OK)
    {
        ROS_WARN("FLIR camera Connect failed : %s", error.GetDescription());
        return false;
    }
    else
    {
        if (pCamera->IsConnected())
        {
            ROS_INFO("FLIR camera Connected");
        }
    }
    error = pCamera->GetCameraInfo(&camInfo);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return false;
    }

    //get the serial no. for the randomly connected camera
    printf("Serial number: %d\n", camInfo.serialNumber);
    printf("Camera model: %s\n", camInfo.modelName);

    printf("Interface Type: %s\n", flycapInterfaceTypeStr[camInfo.interfaceType].c_str());

    printf("Color type: %s\n", camInfo.isColorCamera ? "Colored" : "Monochrome");
    printf("Sensor info: %s\n", camInfo.sensorInfo);
    printf("userDefinedName: %s\n", camInfo.userDefinedName);

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

    error = pCamera->GetConfiguration(&cameraConfig);
    cameraConfig.numBuffers = 5;
    cameraConfig.grabTimeout = FlyCapture2::TIMEOUT_INFINITE;
    cameraConfig.grabMode = FlyCapture2::DROP_FRAMES;
    cameraConfig.highPerformanceRetrieveBuffer = true;
    cameraConfig.asyncBusSpeed = FlyCapture2::BUSSPEED_ANY;
    cameraConfig.isochBusSpeed = FlyCapture2::BUSSPEED_ANY;

    error = pCamera->SetConfiguration(&cameraConfig);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        std::cout << "[#INFO]Error in setCameraConfiguration " << std::endl;
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
    error = rawImage.Convert(format7ImageSettings.pixelFormat, &convertedImage);

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
        tempOut->img = tempimg;
    }

    tempOut->rosheader.stamp.nsec = ts.microSeconds * 1000;
    tempOut->rosheader.stamp.sec = ts.seconds;

    return tempOut;
};

cv_bridge::CvImage *PointGreyCamera::getFrameROS()
{
    cv_bridge::CvImage *rxImg;
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

    std_msgs::Header header;
    rxImg = new cv_bridge::CvImage(header, "");
    header.stamp.nsec = ts.microSeconds * 1000;
    header.stamp.sec = ts.seconds;
    rxImg->encoding = sensor_msgs::image_encodings::BGR8;
    Mat tempimg = cv::Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC3, pdata);
    if (tempimg.empty())
        return NULL;

    if (resizeHalf)
    {
        cv::resize(tempimg, rxImg->image, cv::Size(tempimg.cols / 2, tempimg.rows / 2), 0, 0);
    }
    else
    {
        tempimg.copyTo(rxImg->image);
    }

    return rxImg;
};

bool PointGreyCamera::loadDriverParameters(const FileStorage &fs)
{

    const FileNode &node = fs["pgCam"];
    fsHelper::readOrDefault(node["serialNumber"], (int &)serialNumber, 0);

    fsHelper::readOrDefault(node["resizeHalf"], resizeHalf, false);

    fsHelper::readOrDefault(node["format7_mode"], (int &)format7ImageSettings.mode, (int)FlyCapture2::MODE_0);

    int temp;

    for (int i = 0; i < FlyCapture2::TEMPERATURE; i++)
    {
        node[flycapPropertyTypeNames[i] + "_absControl"] >> temp;
        properties[i].absControl = (temp > 0);
        node[flycapPropertyTypeNames[i] + "_absValue"] >> properties[i].absValue;
        node[flycapPropertyTypeNames[i] + "_autoManualMode"] >> temp;
        properties[i].autoManualMode = (temp > 0);
        node[flycapPropertyTypeNames[i] + "_onOff"] >> temp;
        properties[i].onOff = (temp > 0);
        node[flycapPropertyTypeNames[i] + "_valueA"] >> (int &)properties[i].valueA;
        node[flycapPropertyTypeNames[i] + "_valueB"] >> (int &)properties[i].valueB;
        printf("Read %s from config file:\n", flycapPropertyTypeNames[i].c_str());
        printf("_absControl %s\n", properties[i].absControl ? "Yes" : "No");
        printf("_absValue %f\n", properties[i].absValue);
        printf("_autoManualMode %s\n", properties[i].autoManualMode ? "Yes" : "No");
        printf("_onOff %s\n", properties[i].onOff ? "Yes" : "No");
        printf("_absControl %d\n", properties[i].valueA);
        printf("_absControl %d\n", properties[i].valueB);
    };

    node["format7ImageSettings_mode"] >> (int &)format7ImageSettings.mode;
    node["format7ImageSettings_width"] >> (int &)format7ImageSettings.width;
    node["format7ImageSettings_offsetX"] >> (int &)format7ImageSettings.offsetX;
    node["format7ImageSettings_height"] >> (int &)format7ImageSettings.height;
    node["format7ImageSettings_offsetY"] >> (int &)format7ImageSettings.offsetY;
    string tempstr;
    node["format7ImageSettings_pixelFormat"] >> tempstr;
    for (int i = 0; i < FlyCapture2::NUM_PIXEL_FORMATS; i++)
    {
        if (tempstr == flycapPixelFormatNames[i])
        {
            format7ImageSettings.pixelFormat = (FlyCapture2::PixelFormat)flycapPixelFormatBitFields[i];
            break;
        }
    }
    node["packagesizepercent"] >> packagesizepercent;

    return true;
};

bool PointGreyCamera::storeDriverParameters(FileStorage &fs)
{
    fs << "pgCam"
       << "{"
       << "serialNumber" << (int)serialNumber

       << "resizeHalf" << resizeHalf

       << "format7_mode" << format7ImageSettings.mode;

    for (int i = 0; i < FlyCapture2::TEMPERATURE; i++)
    {
        fs << flycapPropertyTypeNames[i] + "_absControl" << (int)properties[i].absControl
           << flycapPropertyTypeNames[i] + "_absValue" << properties[i].absValue
           << flycapPropertyTypeNames[i] + "_autoManualMode" << (int)properties[i].autoManualMode
           << flycapPropertyTypeNames[i] + "_onOff" << (int)properties[i].onOff
           << flycapPropertyTypeNames[i] + "_valueA" << (int)properties[i].valueA
           << flycapPropertyTypeNames[i] + "_valueB" << (int)properties[i].valueB;
    };

    fs << "format7ImageSettings_mode" << (int)format7ImageSettings.mode
       << "format7ImageSettings_width" << (int)format7ImageSettings.width
       << "format7ImageSettings_offsetX" << (int)format7ImageSettings.offsetX
       << "format7ImageSettings_height" << (int)format7ImageSettings.height
       << "format7ImageSettings_offsetY" << (int)format7ImageSettings.offsetY;

    for (int i = 0; i < FlyCapture2::NUM_PIXEL_FORMATS; i++)
    {
        if (format7ImageSettings.pixelFormat == flycapPixelFormatBitFields[i])
        {
            fs << "format7ImageSettings_pixelFormat" << flycapPixelFormatNames[i];
            break;
        }
    }

    fs << "packagesize" << (int)packagesize
       << "packagesizepercent" << packagesizepercent
       << "}";

    return true;
};

bool PointGreyCamera::setCamConfig()
{
    bool success = true;
    FlyCapture2::Error error;
    for (int i = 0; i < FlyCapture2::TEMPERATURE; i++)
    {
        properties[i].type = (FlyCapture2::PropertyType)i;
        error = pCamera->SetProperty(&properties[i]);
        if (error.GetType() == FlyCapture2::PGRERROR_PROPERTY_NOT_PRESENT)
        {
            continue;
        }
        else if (error.GetType() != FlyCapture2::PGRERROR_OK)
        {

            ROS_WARN("Error setting FLIR Camera Property: \"%s\" : %s",
                     flycapPropertyTypeNames[i].c_str(),
                     error.GetDescription());
            success = false;
        }
        else
        {
            printf("Set flir camera %d  \"%s\" success\n",
                   camInfo.serialNumber,
                   flycapPropertyTypeNames[i].c_str());
        }
    };

    error = pCamera->SetFormat7Configuration(&format7ImageSettings, packagesizepercent);
    if (error.GetType() != FlyCapture2::PGRERROR_OK)
    {
        ROS_WARN("Error setting FLIR Camera format7 configuration : %s",
                 error.GetDescription());
        success = false;
    }
    else
    {
        printf("Set flir camera %d format7 configuration success\n",
               camInfo.serialNumber);
    }
    return success;
};

bool PointGreyCamera::getCamConfig()
{
    bool success = true;
    FlyCapture2::Error error;
    for (int i = 0; i < FlyCapture2::TEMPERATURE; i++)
    {
        success &= getCamProperty((FlyCapture2::PropertyType)i, &properties[i]);
    };

    pCamera->GetFormat7Configuration(&format7ImageSettings, &packagesize, &packagesizepercent);
    printf("Current Format7 settings:\nmode: %d\noffsetX: %d\noffsetY: %d\nheight: %d\nwidth: %d\n",
           (int)format7ImageSettings.mode,
           format7ImageSettings.offsetX,
           format7ImageSettings.offsetY,
           format7ImageSettings.height,
           format7ImageSettings.width);

    printf("pixelFormat:\n");
    for (int i = 0; i < FlyCapture2::NUM_PIXEL_FORMATS; i++)
    {
        if (format7ImageSettings.pixelFormat & flycapPixelFormatBitFields[i])
        {
            printf("%s\n", flycapPixelFormatNames[i].c_str());
            break;
        }
    }

    return success;
};

bool PointGreyCamera::startStream()
{
    printf("startStream %d %d:\n",serialNumber,pCamera);
    FlyCapture2::Error error;
    error = pCamera->StartCapture();
    printf("Started Capture:\n");
    if (error != FlyCapture2::PGRERROR_OK)
    {
        printf("start stream failed: %s\n", error.GetDescription());
    }
    return (error == FlyCapture2::PGRERROR_OK);
};

bool PointGreyCamera::closeStream()
{
    FlyCapture2::Error error = pCamera->StopCapture();
    return (error == FlyCapture2::PGRERROR_OK);
};

void PointGreyCamera::info()
{
    FlyCapture2::PropertyInfo info;
    for (int i = 0; i < FlyCapture2::TEMPERATURE; i++)
    {
        info.type = (FlyCapture2::PropertyType)i;
        pCamera->GetPropertyInfo(&info);
        if (info.present)
        {
            printf("%12s supports: %10s%10s%10s%10s%10s\n",
                   flycapPropertyTypeNames[i].c_str(),
                   info.autoSupported ? "Auto" : "",
                   info.manualSupported ? "Manual" : "",
                   info.absValSupported ? "AbsVal" : "",
                   info.onePushSupported ? "onePush" : "",
                   info.onOffSupported ? "onOff" : "");
            printf("%25s(%5d < x < %5d)",
                   "",
                   info.min,
                   info.max);

            if (info.absValSupported)
            {
                printf("     (%-5f < Abs value < %-5f %12s)",
                       info.absMin,
                       info.absMax,
                       info.pUnits);
            };
            printf("\n");
        }
    };

    FlyCapture2::Format7Info f7info;
    bool f7supported;
    for (int i = 0; i < FlyCapture2::NUM_MODES; i++)
    {
        f7info.mode = (FlyCapture2::Mode)i;
        pCamera->GetFormat7Info(&f7info, &f7supported);
        if (f7supported)
        {
            printf(
                "Format7 mode: %d\n"
                "imageHStepSize: %d\n"
                "imageVStepSize: %d\n"
                "maxHeight: %d\n"
                "maxWidth: %d\n"
                "minPacketSize: %d\n"
                "packetSize: %d\n"
                "maxPacketSize: %d\n"
                "offsetHStepSize: %d\n"
                "offsetVStepSize: %d\n"
                "percentage: %f\n"
                "pixelFormatBitField: %#x\n"
                "vendorPixelFormatBitField: %#x\n\n",
                f7info.mode,
                f7info.imageHStepSize,
                f7info.imageVStepSize,
                f7info.maxHeight,
                f7info.maxWidth,
                f7info.minPacketSize,
                f7info.packetSize,
                f7info.maxPacketSize,
                f7info.offsetHStepSize,
                f7info.offsetVStepSize,
                f7info.percentage,
                f7info.pixelFormatBitField,
                f7info.vendorPixelFormatBitField);

            printf("Supported formats:\n");
            for (int i = 0; i < FlyCapture2::NUM_PIXEL_FORMATS; i++)
            {
                if (f7info.pixelFormatBitField & flycapPixelFormatBitFields[i])
                {
                    printf("%s\n", flycapPixelFormatNames[i].c_str());
                }
            }
        }
    }
};

bool PointGreyCamera::getCamProperty(
    FlyCapture2::PropertyType type,
    FlyCapture2::Property *prop)
{
    prop->type = type;
    FlyCapture2::Error error = pCamera->GetProperty(prop);

    if (error != FlyCapture2::PGRERROR_OK)
    {
        ROS_WARN("Error getting FLIR Camera Property: \"%s\"\n", flycapPropertyTypeNames[type].c_str());
        error.PrintErrorTrace();
        return false;
    }
    else if (prop->present)
    {
        printf("Retrieved FLIR Camera Property: %15s", flycapPropertyTypeNames[type].c_str());
        printf("%10s", prop->onOff ? "Enabled" : "Disabled");
        printf("%10s", prop->autoManualMode ? "Auto" : "Manual");
        printf("\tAbsolute value: %f", prop->absValue);
        printf("\tValue A: %d\tValue B: %d\n", prop->valueA, prop->valueB);
    }
    return true;
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
        ROS_INFO("FLIR Camera %d temperature: %f C", camInfo.serialNumber, fProp.absValue * 100.0 - 273.15);
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
