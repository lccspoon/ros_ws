// #include "imuServer.h"
#include "imu_main.h"
#include "../../robot_master/robot_main.h"
#if HARD_WARE==1  //lcc 20230626



// #define USE_XSENS

 exlcm::imu_type _vectorNavData;

// int imu_main();
// void runXsens(void);
// void initialXsens(void);


void imumain(void)
{

  #if HARD_WARE==1

    //初始化  xsens数据
    initialXsens();

    //circle loop
    while(1){ 
      ///////////////////////////////
      runXsens();

      // std::cout<<" _vectorNavData.acc"<<std::endl;
      // std::cout<< _vectorNavData.acc<<std::endl;

      // printf(" _vectorNavData.acc: %f %f %f \n",_vectorNavData.acc[0],_vectorNavData.acc[1],_vectorNavData.acc[2]);
      // printf(" _vectorNavData.gyr: %f %f %f \n",_vectorNavData.gyr[0],_vectorNavData.gyr[1],_vectorNavData.gyr[2]);
      // printf(" _vectorNavData.euler: %f %f %f \n",_vectorNavData.euler[0],_vectorNavData.euler[1],_vectorNavData.euler[2]);
      // printf("\n--next--\n");

      // std::cout<<" _vectorNavData.euler"<<std::endl;
      // std::cout<< _vectorNavData.euler <<std::endl;

      // std::cout<<" _vectorNavData.gyr"<<std::endl;
      // std::cout<< _vectorNavData.gyr<<std::endl;


      usleep(2000);
    }
  #endif

}



void runXsens(void)
{
  #if HARD_WARE==1
        // std::cout<<"运行 xsens "<<std::endl;

      //运行 xsens
        Xsens_run();
        exlcm::imu_type *vectorNav = get_vectorNav_data();
      #ifdef USE_XSENS    
        memcpy(&_vectorNavData, vectorNav, sizeof(_vectorNavData));
      #endif

#endif

}


void initialXsens(void)
{
  #if HARD_WARE==1
      //初始化  xsens数据
          #ifdef USE_XSENS
            printf("[Hardware] Init vectornav\n");
            if (Xsens_init())
            {
              printf("Vectornav failed to initialize\n");
            }
          #endif
  #endif
}

#endif