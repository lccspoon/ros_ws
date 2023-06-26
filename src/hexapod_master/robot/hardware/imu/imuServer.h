#ifndef _IMUSERVER_H
#define _IMUSERVER_H
#include <cstring>
#include <xscontrol_def.h>
#include <xsdevice_def.h>
#include <xsscanner.h>
#include <xsoutputconfigurationarray.h>
#include <xsdatapacket.h>
#include <xstime.h>
#include <xsens_mutex.h>
// #include "../../../xspublic/include/xspublic/xscontrol_def.h"
// #include "../../../xspublic/include/xspublic/xsdevice_def.h"
// #include "../../../xspublic/include/xspublic/xsscanner.h"
// #include "../../../xspublic/include/xspublic/xsoutputconfiguration.h"
// #include "../../../xspublic/include/xspublic/xsdatapacket.h""
// #include "../../../xspublic/include/xspublic/xstime.h"
// #include "../../../xspublic/include/xspublic/xsens_mutex.h"



#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include <stdio.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include "STRUCT.h"
#include "imu_type.hpp"
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>

class CallbackHandler : public XsCallback
{
public:
    CallbackHandler(size_t maxBufferSize = 5)
        : m_maxNumberOfPacketsInBuffer(maxBufferSize), m_numberOfPacketsInBuffer(0)
    {
    }

    virtual ~CallbackHandler() throw()
    {
    }

    bool packetAvailable() const
    {
        xsens::Lock locky(&m_mutex);
        return m_numberOfPacketsInBuffer > 0;
    }

    XsDataPacket getNextPacket()
    {
        assert(packetAvailable());
        xsens::Lock locky(&m_mutex);
        XsDataPacket oldestPacket(m_packetBuffer.front());
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        return oldestPacket;
    }

protected:
    void onLiveDataAvailable(XsDevice *, const XsDataPacket *packet) override
    {
        xsens::Lock locky(&m_mutex);
        assert(packet != 0);
        while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
            (void)getNextPacket();

        m_packetBuffer.push_back(*packet);
        ++m_numberOfPacketsInBuffer;
        assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
    }

private:
    mutable xsens::Mutex m_mutex;

    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    list<XsDataPacket> m_packetBuffer;
};
int Xsens_init();
void Xsens_run();
int Xsens_close();
exlcm::imu_type *get_vectorNav_data();
#endif
