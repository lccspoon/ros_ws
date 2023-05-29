// #include "imuServer.h"
#include "imu_main.h"

// #define USE_XSENS

 exlcm::imu_type _vectorNavData;

// int imu_main();
// void runXsens(void);
// void initialXsens(void);


int imu_main(void)
{
  //初始化  xsens数据
  initialXsens();

  //circle loop
  while(1){ 
    ///////////////////////////////
    runXsens();
    
    // // check the data
    // std::cout<<"Acc: "<<std::endl;
    // for (int i = 0; i < 3; i++)
    // {
    //   std::cout<<_vectorNavData.acc[i]<<" ";
    // }
    // std::cout<<std::endl;

    // std::cout<<"Gyr: "<<std::endl;
    // for (int i = 0; i < 3; i++)
    // {
    //   std::cout<<_vectorNavData.gyr[i]<<" ";
    // }
    // std::cout<<std::endl;
    
    ///////////////////////////////
    
    usleep(150000);
  }
  return 0;

}


void runXsens(void)
{
  // std::cout<<"运行 xsens "<<std::endl;

//运行 xsens
  Xsens_run();
  exlcm::imu_type *vectorNav = get_vectorNav_data();
#ifdef USE_XSENS    
  memcpy(&_vectorNavData, vectorNav, sizeof(_vectorNavData));
#endif

}


void initialXsens(void)
{
//初始化  xsens数据
  #ifdef USE_XSENS
    printf("[Hardware] Init vectornav\n");
    if (Xsens_init())
    {
      printf("Vectornav failed to initialize\n");
    }
  #endif
}