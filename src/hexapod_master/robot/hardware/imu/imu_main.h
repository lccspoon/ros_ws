#include "imuServer.h"
#include "imu_type.hpp"

#define USE_XSENS

extern exlcm::imu_type _vectorNavData;

int imu_main(void);
void runXsens(void);
void initialXsens(void);
