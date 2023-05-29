#include "imuServer.h"

#define pi 3.1415926
#define MAX_LINE 15
using namespace std;

extern pthread_mutex_t UserInput_mutex;
// extern FLAG_CONTROL FlagTime;
exlcm::imu_type vectorNav_Data;
double rroll, rpitch, ryaw;
Journaller *gJournal = 0;
XsControl *control = nullptr;
XsDevice *device = nullptr;
XsPortInfo mtPort;		  // Find an MTi device
CallbackHandler callback; // Create and attach callback handler to device
						  //--------------------------------------------------------------------------------
// Lambda function for error handling
auto handleError = [=](string errorString)
{
	control->destruct();
	cout << errorString << endl;
	cout << "Press [ENTER] to continue." << endl;
	cin.get();
	return -1;
};
int Xsens_init()
{
	//--------------------------------------------------------------------------------
	cout << "Creating XsControl object..." << endl;
	XsControl *control = XsControl::construct();
	assert(control != 0);

	cout << "Scanning for devices..." << endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found. Aborting.");

	cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

	cout << "Opening port..." << endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");

	// Get the device object
	XsDevice *device = control->device(mtPort.deviceId());
	assert(device != 0);

	cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	cout << "Putting device into configuration mode..." << endl;
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");

	cout << "Configuring the device..." << endl;

	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	device->readEmtsAndDeviceConfiguration();

	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

	if (device->deviceId().isImu())
	{
		cout << "yes it is acc" << string(40, '-') << endl;
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 400));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
	}
	else if (device->deviceId().isVru() || device->deviceId().isAhrs())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
		// xzb230522  配置 
		configArray.push_back(XsOutputConfiguration(XDI_AccelerationHR, 500));//XDI_AccelerationHR
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurnHR, 500));//XDI_RateOfTurnHR
	}
	else if (device->deviceId().isGnss())
	{
		cout << "yes it is" << string(40, '-') << endl;
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 400));
		configArray.push_back(XsOutputConfiguration(XDI_LatLon, 100));
		configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
		configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 100));
	}
	else
	{
		return handleError("Unknown device while configuring. Aborting.");
	}

	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");

	cout << "Creating a log file..." << endl;
	string logFileName = "logfile.mtb";
	if (device->createLogFile(logFileName) != XRV_OK)
		return handleError("Failed to create a log file. Aborting.");
	else
		cout << "Created a log file: " << logFileName.c_str() << endl;

	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

	cout << "Starting recording..." << endl;
	if (!device->startRecording())
		return handleError("Failed to start recording. Aborting.");

	cout << "\nMain loop. Recording data." << endl;
	cout << string(79, '-') << endl;
	return 0;
}
void Xsens_run()
{
	int64_t startTime = XsTime::timeStampNow();

	if (callback.packetAvailable())
	{
		cout << setw(5) << fixed << setprecision(2);

		// Retrieve a packet
		XsDataPacket packet = callback.getNextPacket();
		if (packet.containsCalibratedData())
		{
			XsVector acc = packet.calibratedAcceleration();
			cout << "\r"
				 << "Acc X:" << acc[0]
				 << ", Acc Y:" << acc[1]
				 << ", Acc Z:" << acc[2];
			vectorNav_Data.acc[0] = acc[0];
			vectorNav_Data.acc[1] = acc[1];
			vectorNav_Data.acc[2] = acc[2];

			XsVector gyr = packet.calibratedGyroscopeData();
			cout << " |Gyr X:" << gyr[0]
				 << ", Gyr Y:" << gyr[1]
				 << ", Gyr Z:" << gyr[2];
			vectorNav_Data.gyr[0]=gyr[0];
			vectorNav_Data.gyr[1]=gyr[1];
			vectorNav_Data.gyr[2]=gyr[2];


			// XsVector mag = packet.calibratedMagneticField();
			// cout << " |Mag X:" << mag[0]
			// 	 << ", Mag Y:" << mag[1]
			// 	 << ", Mag Z:" << mag[2];
		}

		// xzb230522
		if (packet.containsAccelerationHR())
		{
			XsVector accHR = packet.accelerationHR();
			cout << "\r"
				 << "AccHR X:" << accHR[0]
				 << ", AccHR Y:" << accHR[1]
				 << ", AccHR Z:" << accHR[2];
			vectorNav_Data.acc[0] = accHR[0];
			vectorNav_Data.acc[1] = accHR[1];
			vectorNav_Data.acc[2] = accHR[2];

		}
		if (packet.containsRateOfTurnHR())
		{
			XsVector gyrHR = packet.rateOfTurnHR();
			cout << " |GyrHR X:" << gyrHR[0]
				 << ", GyrHR Y:" << gyrHR[1]
				 << ", GyrHR Z:" << gyrHR[2];
			vectorNav_Data.gyr[0]=gyrHR[0];
			vectorNav_Data.gyr[1]=gyrHR[1];
			vectorNav_Data.gyr[2]=gyrHR[2];
		}
		// xzb230522


		if (packet.containsOrientation())
		{
			XsQuaternion quaternion = packet.orientationQuaternion();
			cout << "\r"
				 << "q0:" << quaternion.w()
				 << ", q1:" << quaternion.x()
				 << ", q2:" << quaternion.y()
				 << ", q3:" << quaternion.z();
			vectorNav_Data.orientation[0] = quaternion.w();
			vectorNav_Data.orientation[1] = quaternion.x();
			vectorNav_Data.orientation[2] = quaternion.y();
			vectorNav_Data.orientation[3] = quaternion.z();

			XsEuler euler = packet.orientationEuler();
			cout << " |Roll:" << euler.roll()
				 << ", Pitch:" << euler.pitch()
				 << ", Yaw:" << euler.yaw();
			// pthread_mutex_lock(&UserInput_mutex);

			rroll = euler.roll() * pi / 180;
			rpitch = euler.pitch() * pi / 180;
			ryaw = euler.yaw() * pi / 180;
			vectorNav_Data.euler[0] = euler.roll() * pi / 180;
			vectorNav_Data.euler[1] = euler.pitch() * pi / 180;
			vectorNav_Data.euler[2] = euler.yaw() * pi / 180;
			// vectorNav_Data.euler[2] = 0;
			// ryaw = 0;
			// rroll=0;
			// rpitch=0;
			// pthread_mutex_unlock(&UserInput_mutex);
		}
		if (packet.containsRawAcceleration())
		{
			XsVector rvelocity = packet.latitudeLongitude();
			cout << " |vel:" << rvelocity[0]
				 << ", vel:" << rvelocity[1];
		}
		if (packet.containsLatitudeLongitude())
		{
			XsVector latLon = packet.latitudeLongitude();
			cout << " |Lat:" << latLon[0]
				 << ", Lon:" << latLon[1];
		}

		if (packet.containsAltitude())
			cout << " |Alt:" << packet.altitude();

		if (packet.containsVelocity())
		{
			XsVector vel = packet.velocity(XDI_CoordSysEnu);
			cout << " |E:" << vel[0]
				 << ", N:" << vel[1]
				 << ", U:" << vel[2];
		}
		int64_t endTime = XsTime::timeStampNow();
		cout << flush;
	}
}

int Xsens_close()
{
	cout << "Stopping recording..." << endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.");

	cout << "Closing log file..." << endl;
	if (!device->closeLogFile())
		return handleError("Failed to close log file. Aborting.");

	cout << "Closing port..." << endl;
	control->closePort(mtPort.portName().toStdString());

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

	cout << "Successful exit." << endl;

	cout << "Press [ENTER] to continue." << endl;
	cin.get();
	return 0;
}
exlcm::imu_type *get_vectorNav_data()
{
	return &vectorNav_Data;
}
