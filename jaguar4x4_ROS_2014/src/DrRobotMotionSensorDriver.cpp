/*
 * DrRobotMotionSensorDriver.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: dri
 */
/*
std::string str;
std::vector<char> writable(str.size() + 1);
std::copy(str.begin(), str.end(), writable.begin());

string ( const char * s, size_t n );
string ( const string& str, size_t pos, size_t n = npos );

*/



#include "DrRobotMotionSensorDriver.hpp"

//#define DEBUG_ERROR           //set printf out error message
#undef DEBUG_ERROR


using namespace std;
using namespace DrRobot_MotionSensorDriver;

double DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::resTable[25] = {114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,
                        7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
double DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::tempTable[25] = { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };


/*! this function is construct function for DrRobotMotionSensorDriver Class
   It will initialize all the internal variables
*/

DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::DrRobotMotionSensorDriver()
{

  _robotConfig = new DrRobotMotionConfig();
  _robotConfig->commMethod = Network;
  sprintf(_robotConfig->serialPortName, "ttyS0");

  _robotConfig->portNum = 10001;
  sprintf(_robotConfig->robotID, "DrRobot");
  sprintf(_robotConfig->robotIP, "192.168.0.60");

  _robotConfig->robotType = Puma;

  _nMsgLen = 0;

  bzero(&_addr, sizeof(_addr));
  _addr.sin_family = AF_INET;
  _addr.sin_port = htons(_robotConfig->portNum);
  _addr_len = sizeof _addr;
  _numbytes = 0;

  //udp, not used now
  //_sockfd = socket(AF_INET, SOCK_DGRAM, 0);

  //TCP
  if ((_sockfd = socket(PF_INET, SOCK_STREAM,IPPROTO_TCP)) < 0)
  {
      printf("Failed to createsocket");
  }
  _serialfd = -1;

  _tv.tv_sec = 0;
  _tv.tv_usec = 5;             //200us ?

  _stopComm = true;
  _comCnt = 0;
  _mutex_Data_Buf = PTHREAD_MUTEX_INITIALIZER;
  _eCommState = Disconnected;
 
}

DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::~DrRobotMotionSensorDriver()
{
  if (portOpen())
    close();
}

bool DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::portOpen()
{
  if ( (_eCommState == Connected) && (!_stopComm))
    return true;
  else
    return false;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::close()
{

  _stopComm = true;
  _pCommThread->join();
  _eCommState = Disconnected;

  //for UDP , do we need close socket?

  if (_robotConfig->commMethod == Network)
  {
    if (_sockfd > 0)
    {
      ::close(_sockfd);
      _sockfd = -1;
    }
  }
  else if(_robotConfig->commMethod == Serial)
  {
    if (_serialfd > 0)
      {
        :: close(_serialfd);
        _serialfd = -1;
      }
  }
}


int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::openSerial(const char* serialPort, const long BAUD)
{

  if (portOpen())
    close();

  _robotConfig->commMethod = Serial;

  sprintf(_robotConfig->serialPortName, "%s",serialPort);

  _serialfd = ::open(_robotConfig->serialPortName, O_RDWR | O_NONBLOCK | O_NOCTTY);
  //_serialfd = ::open("/dev/ttyS0", O_RDWR | O_NONBLOCK | O_NOCTTY);
  if (_serialfd > 0)
  {
    struct termios newtio;

    tcgetattr(_serialfd, &newtio);
    memset(&newtio.c_cc, 0,sizeof(newtio.c_cc));
    newtio.c_cflag = BAUD|CS8|CLOCAL|CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN] = 0;              //VMIN = 0, VTIME = 0, read will return immediately
    newtio.c_cc[VTIME] = 0;
    tcflush(_serialfd,TCIFLUSH);
    tcsetattr(_serialfd,TCSANOW, &newtio);

    printf("listener: waiting for robot server, starting receiving...\n");
   _eCommState = Connected;
   _stopComm = false;
   _pCommThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DrRobotMotionSensorDriver::commWorkingThread, this)));
     return 0;
 }

  else
  {
    const char *extra_msg = "";
    switch(errno)
    {
      case EACCES:
          extra_msg = "You probably don't have permission to open the port for reading and writing.\n";
          debug_ouput(extra_msg);
          break;
      case ENOENT:
        extra_msg = "The request port does not exit. Was the port name misspelled?\n";
        debug_ouput(extra_msg);
        break;
    }
   _stopComm = true;
   _eCommState = Disconnected;
    return errno;
  }


}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::debug_ouput(const char* errorstr)
{
#ifdef DEBUG_ERROR
    printf("DrRobot Motion Sensor: %s", errorstr);
#endif
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::vali_ip(const char*  ip_str)
{
  unsigned int n1,n2,n3,n4;
  if ( sscanf(ip_str, "%u.%u.%u.%u", &n1,&n2,&n3,&n4) != 4 ) return 1;
  if ((n1 != 0) && (n1 <= 255) && (n2 <= 255) && (n3 <= 255) && (n4 <= 255) )
  {
    char buf[64];
    sprintf(buf,"%u.%u.%u.%u", n1,n2,n3,n4);
    if (strcmp(buf,ip_str)) return 1;
    return 0;
  }

  return 1;
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::openNetwork(const char* robotIP, const int portNum )
{
  char temp[512] ;
  //check the parameter first
  if (portNum <= 0)
  {

    debug_ouput(temp);
    return -1;
  }

  if (vali_ip(robotIP) == 1)
  {
    sprintf(temp, "DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", robotIP);
    debug_ouput(temp);
    return -2;
  }
  _robotConfig->commMethod = Network;
  _robotConfig->portNum = portNum;

  sprintf(_robotConfig->robotIP, "%s",robotIP);
  bzero(&_addr, sizeof(_addr));
  _addr.sin_family = AF_INET;
  _addr.sin_port = htons(_robotConfig->portNum);

  if ( inet_aton(_robotConfig->robotIP, &_addr.sin_addr) == 0)
  {
    sprintf(temp, "DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", _robotConfig->robotIP);
    debug_ouput(temp);
    return -3;
  }

  _stopComm = false;

  //TCP, setup connection here
  if (connect(_sockfd, (struct sockaddr *) &_addr,sizeof(_addr)) < 0) {
    printf("Failed to connect with robot. IP address %s, Port: %d \n", _robotConfig->robotIP, _robotConfig->portNum);
    _stopComm = true;
   return -1;
   }


  // for UDP, need send something out first to setup the communication
  /*
 _numbytes = sendAck();
 if (_numbytes < 0)
 {
   _stopComm = true;
   perror("sendto");
   return -4;
 }
*/
 printf("listener: waiting for robot server, starting receiving...\n");
 _eCommState = Connected;
_stopComm = false;
 _pCommThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DrRobotMotionSensorDriver::commWorkingThread, this)));
 return 0;
}

//communication thread here
void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::commWorkingThread(){
    while(!_stopComm)
    {
       if (_robotConfig->commMethod == Network)
       {

        FD_ZERO(&_readfds);
        FD_SET(_sockfd, &_readfds);
        int retval = select(_sockfd + 1, &_readfds, NULL, NULL, &_tv);
        if(FD_ISSET(_sockfd,&_readfds))
        {
          //TCP
          if ((_numbytes = recv(_sockfd, _recBuf, MAXBUFLEN-1 , 0)) < 0)
          {
                  perror("recv");
                  return;
          }
          #ifdef DEBUG_ERROR
              printf("listener: packet is %d bytes long\n", _numbytes);
          #endif

            //UDP, not used now
          /*
          if ((_numbytes = recvfrom(_sockfd, _recBuf, MAXBUFLEN-1 , 0,(struct sockaddr *)&_addr, &_addr_len)) == -1)
          {
                  perror("recvfrom");
                  return;
          }
          #ifdef DEBUG_ERROR
              printf("listener: packet is %d bytes long\n", _numbytes);
          #endif
          */
          _comCnt = 0;
/*	  
	  char *data = new char[_numbytes + 1];
	  std::strncpy(data,_recBuf,_numbytes);
   	  printf("Got:: %s",data);
*/	  
          handleComData(_recBuf,_numbytes);
        }
        else
        {
          _comCnt++;
          usleep(20);              //ms (Kevin Kongmanychanh) This was originally 2000 ms
          if (_comCnt > COMM_LOST_TH)
          {
            printf("Communication is lost, need close all. IP address %s, Port: %d \n", _robotConfig->robotIP, _robotConfig->portNum);
            _stopComm = true;
            return;
          }
        }
       }
      else if(_robotConfig->commMethod == Serial)
      {

        _numbytes = read(_serialfd,_recBuf, sizeof(_recBuf));
        if (_numbytes <= 0 )   //( (_numbytes == -1) && (errno != EAGAIN) && (errno != EWOULDBLOCK) )
        {
          //read erro,
          _comCnt ++;
          //printf ("Serial time out\n");
          usleep(10000);
          if (_comCnt > COMM_LOST_TH)
          {
            printf("Communication is lost, need close all. Serial Port is %s \n", _robotConfig->serialPortName);
            _stopComm = true;
            _eCommState = Disconnected;
            ::close(_serialfd);
            _serialfd = -1;
          }

        }
        else
        {
      #ifdef DEBUG_ERROR
         printf("listener: packet is %d bytes long\n", _numbytes);
      #endif
         _comCnt = 0;
         handleComData(_recBuf,_numbytes);
        }
       }
    }
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::DealWithPacket(const char *lpComData, const int nLen)
{
  char *data = new char[nLen + 1];
  std::strncpy(data,lpComData,nLen-2);

 // printf("Packet is : %s\n",data);
  if  (data[0] == '#')
  {

      debugCommMessage("IMU sensor data package!\n");
      if ( (nLen -1) > 0)
	{
        	processIMUMessage(data + 1,nLen -1);
	}
  }
  else if  (data[0] == '$')
  {
      debugCommMessage("GPS sentence package!\n");
      // start with $GPRMC,
      if ( (nLen - 7) > 0)
        processGPSMessage(data + 7, nLen -7 );
  }
  else if  (data[0] == 'M')
  {
      debugCommMessage("Motor sensor/Motor Driver board data package! \n");

      processMotorMessage(data, nLen);
  }
  data = NULL;
  delete(data);
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::processIMUMessage(char * pData, const int nLen)
{
    char *testMsg;
    int msgPtr = 0;
    //sample messge will like this
    // belwo format not used now 135,PITCH,0.02,ROLL,0.01,YAW,1.39,GYRO,2,10,-38,ACC,-4,1,245,COMP,85,381,-501,
// now use
//127,YAW,1.39,GYRO,2,10,-30,ACC,-4,1,245,COMP,85,381,-501
    testMsg = strtok(pData,",");
    _imuSensorData.pitch = 0;
    _imuSensorData.roll = 0;
while(testMsg != NULL)
    {
        if (msgPtr == 0)
        {
            _imuSensorData.seq = atoi(testMsg);
		
        }
        else if(msgPtr == 2)
        {
            _imuSensorData.yaw = atof(testMsg);
        }
        else if(msgPtr == 4)
        {
            _imuSensorData.gyro_x = atoi(testMsg);
        }
        else if(msgPtr == 5)
        {
            _imuSensorData.gyro_y = atoi(testMsg);
        }
        else if(msgPtr == 6)
        {
            _imuSensorData.gyro_z = atoi(testMsg);
        }else if(msgPtr == 8)
        {
            _imuSensorData.accel_x = atoi(testMsg);
        }else if(msgPtr == 9)
        {
            _imuSensorData.accel_y = atoi(testMsg);
        }else if(msgPtr == 10)
        {
            _imuSensorData.accel_z = atoi(testMsg);
        }else if(msgPtr == 12)
        {
            _imuSensorData.comp_x = atoi(testMsg);
        }else if(msgPtr == 13)
        {
            _imuSensorData.comp_y = atof(testMsg);
        }else if(msgPtr == 14)
        {
            _imuSensorData.comp_z = atoi(testMsg);
	
        }

        testMsg = strtok(NULL,",");
        msgPtr++;
    }

/*
    while(testMsg != NULL)
    {
        if (msgPtr == 0)
        {
            _imuSensorData.seq = atoi(testMsg);
		
        }
        else if(msgPtr == 2)
        {
            _imuSensorData.pitch = atof(testMsg);
        }
        else if(msgPtr == 4)
        {
            _imuSensorData.roll = atof(testMsg);
        }
        else if(msgPtr == 6)
        {
            _imuSensorData.yaw = atof(testMsg);
        }
        else if(msgPtr == 8)
        {
            _imuSensorData.gyro_x = atoi(testMsg);
        }
        else if(msgPtr == 9)
        {
            _imuSensorData.gyro_y = atoi(testMsg);
        }
        else if(msgPtr == 10)
        {
            _imuSensorData.gyro_z = atoi(testMsg);
        }else if(msgPtr == 12)
        {
            _imuSensorData.accel_x = atoi(testMsg);
        }else if(msgPtr == 13)
        {
            _imuSensorData.accel_y = atoi(testMsg);
        }else if(msgPtr == 14)
        {
            _imuSensorData.accel_z = atoi(testMsg);
        }else if(msgPtr == 16)
        {
            _imuSensorData.comp_x = atoi(testMsg);
        }else if(msgPtr == 17)
        {
            _imuSensorData.comp_y = atof(testMsg);
        }else if(msgPtr == 18)
        {
            _imuSensorData.comp_z = atoi(testMsg);
	
        }

        testMsg = strtok(NULL,",");
        msgPtr++;
    }
*/
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::processGPSMessage(char * pData, const int nLen)
{
    char* testMsg;
    int msgPtr= 0;
    //sample messge will like this
    //$GPRMC,003625.2,V,4351.29884,N,07921.64936,W,,,220899,010.6,W*7F
	
    testMsg = strtok(pData,",");
    while(testMsg != NULL)
    {
        if (msgPtr == 0)
        {
            _gpsSensorData.timeStamp = atoi(testMsg);
        }
        else if(msgPtr == 1)
        {
            if ( strcmp(testMsg,"A") == 0)
            {
                _gpsSensorData.gpsStatus = 0;
            }
            else if(strcmp(testMsg,"V") == 0)
            {
                _gpsSensorData.gpsStatus = -1;
            }
	    
   
       }
        else if(msgPtr == 2)
        {
            _gpsSensorData.latitude = trans2Degree(atof(testMsg));
	    
        }
        else if(msgPtr == 3)
        {
            if(strcmp(testMsg,"N") == 0)
            {
                lathem = 1;
            }
            else if(strcmp(testMsg, "S") == 0)
            {
                lathem = -1;
            }
        }
        else if(msgPtr == 4)
        {
            _gpsSensorData.longitude = trans2Degree(atof(testMsg));
        }
        else if(msgPtr == 5)
        {
            if (strcmp(testMsg,"E") == 0)
            {
                longhem = 1;
            }
            else if(strcmp(testMsg,"W") == 0)
            {
                longhem = -1;
            }

        }
        else if(msgPtr == 6)
        {
            if(_gpsSensorData.gpsStatus == 0)
            {
                _gpsSensorData.vog = atof(testMsg) * KNNOT2MS;

            }
            else
            {
                _gpsSensorData.vog = 0;
		_gpsSensorData.dateStamp = atoi(testMsg);
                
            }

        }else if(msgPtr == 7)
        {
            if(_gpsSensorData.gpsStatus == 0)
            {
                _gpsSensorData.cog = atof(testMsg);
            }
            else
            {
                _gpsSensorData.cog = 0;
            }

        }else if(msgPtr == 8)
        {
 	    if(_gpsSensorData.gpsStatus == 0)
            {
                _gpsSensorData.dateStamp = atoi(testMsg);
	     }
	
        }
        testMsg = strtok(NULL,",");
        msgPtr++;
	
    }

    _gpsSensorData.latitude *= lathem;
    _gpsSensorData.longitude *= longhem;

}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::processMotorMessage(char * pData, const int nLen)
{
    int index = -1;
    double temp = 0;
    int msgPtr = 0;
    char* testMsg;
    //MMx
    if (  pData[2] == '0')
        index = 0;
    else if (  pData[2] == '1')
        index = 1;
    else if (  pData[2] == '2')
        index = 2;
    else if (  pData[2] == '3')
        index = 3;
    else
        index = -1;


    if (index >= 0)
    {
        //decode motor driver board data here

        if ((pData[4] == 'A') && (pData[5] == '='))       //motor current
        {
            testMsg = strtok(pData + 6,":");
            while(testMsg != NULL)
            {
                if (msgPtr == 0)
                {
                    temp = atof(testMsg);
                    _motorSensorData.motorSensorCurrent[2*index] = temp / 10;
                }
                else if(msgPtr == 1)
                {
                    temp = atof(testMsg);
                    _motorSensorData.motorSensorCurrent[2*index + 1] = temp / 10;
                }
                testMsg = strtok(NULL,":");
                msgPtr++;
            }
        }
        else if ((pData[4] == 'A') && (pData[5] == 'I') && (pData[6] == '='))       //motor temperature from ai3,ai4
        {

            testMsg = strtok(pData + 7,":");
            while(testMsg != NULL)
            {
                if (msgPtr == 2)
                {
                    temp = atof(testMsg);
                    _motorSensorData.motorSensorTemperature[2*index] = trans2Temperature(temp);
                }
                else if(msgPtr == 3)
                {
                    temp = atof(testMsg);
                    _motorSensorData.motorSensorTemperature[2*index + 1]= trans2Temperature(temp);
                }
                testMsg = strtok(NULL,":");
                msgPtr++;
            }
        }
        else if ((pData[4] == 'C') && (pData[5] == '='))       //motor encoder position count
        {

            testMsg = strtok(pData + 6,":");
            while(testMsg != NULL)
            {
                if (msgPtr == 0)
                {
                    _motorSensorData.motorSensorEncoderPos[2*index] = atoi(testMsg);
                }
                else if(msgPtr == 1)
                {
                    _motorSensorData.motorSensorEncoderPos[2*index + 1]= atoi(testMsg);
                }
                testMsg = strtok(NULL,":");
                msgPtr++;
            }
        }
        else if ((pData[4] == 'D') && (pData[5] == '='))       //digital input, not used now
        {
            _motorBoardData.dinput[index] = atoi(pData + 6);
        }
        else if ((pData[4] == 'D') && (pData[5] == 'O') && (pData[7] == '='))       //digital output, not used now
        {
            _motorBoardData.doutput[index]= atoi(pData + 8);
        }
        else if ((pData[4] == 'P') && (pData[5] == '='))       //motor output power -1000~ 1000
        {
            testMsg = strtok(pData + 6,":");
            while(testMsg != NULL)
            {
                if (msgPtr == 0)
                {
                    _motorSensorData.motorSensorPWM[2*index] = atoi(testMsg);
                }
                else if(msgPtr == 1)
                {
                    _motorSensorData.motorSensorPWM[2*index + 1] = atoi(testMsg);
                }
                testMsg = strtok(NULL,":");
                msgPtr++;
            }
        }
        else if ((pData[4] == 'S') && (pData[5] == '='))       //motor encoder velocity in RPM
        {
            testMsg = strtok(pData + 6,":");
            while(testMsg != NULL)
            {
                if (msgPtr == 0)
                {
                    _motorSensorData.motorSensorEncoderVel[2*index]= atoi(testMsg);

                }
                else if(msgPtr == 1)
                {
                    _motorSensorData.motorSensorEncoderVel[2*index + 1]= atoi(testMsg);
                }
                testMsg = strtok(NULL,":");
                msgPtr++;
            }
        }
        else if ((pData[4] == 'T') && (pData[5] == '='))       //motor driver board temperature,
        {

            testMsg = strtok(pData + 6,":");
            while(testMsg != NULL)
            {
                if (msgPtr == 0)
                {
                   _motorBoardData.temp2[index] = atof(testMsg);
                }
                else if(msgPtr == 1)
                {
                    _motorBoardData.temp3[index]= atof(testMsg);

                }
                testMsg = strtok(NULL,":");
                msgPtr++;
            }
        }
        else if ((pData[4] == 'V') && (pData[5] == '='))       //motor driver board power voltage reading
        {

            testMsg = strtok(pData + 6,":");
            while(testMsg != NULL)
            {
                if (msgPtr == 0)
                {
                    _motorBoardData.vol12V[index]= atof(testMsg)/10;
                }
                else if(msgPtr == 1)
                {
                    _motorBoardData.volMain[index]= atof(testMsg)/10;
                }
                else if(msgPtr == 2)
                {
                    _motorBoardData.vol5V[index]= atof(testMsg)/1000;
                }

                testMsg = strtok(NULL,":");
                msgPtr++;
            }
        }
        else if ((pData[4] == 'C') && (pData[5] == 'R') && (pData[6] == '='))       //motor encoder position different count
        {
            testMsg = strtok(pData + 7,":");
            while(testMsg != NULL)
            {
                if (msgPtr == 0)
                {
                    _motorSensorData.motorSensorEncoderPosDiff[2*index]= atoi(testMsg);

                }
                else if(msgPtr == 1)
                {
                    _motorSensorData.motorSensorEncoderPosDiff[2*index + 1] = atoi(testMsg);
                }
                testMsg = strtok(NULL,":");
                msgPtr++;
            }
        }
        else if ((pData[4] == 'F') && (pData[5] == 'F') && (pData[6] == '='))       //motor driver board status
        {
            _motorBoardData.status[index]= atoi(pData + 7);
                  }
        else if (pData[4] == '+')    // valid command received
        {
            _motorBoardData.ack[index] = 1;

        }
        else if (pData[4] == '-')      //invalid command received
        {
            _motorBoardData.ack[index] = 0;
        }
    }

}

double DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::trans2Degree(double angle)
{
    double deg = 0;

    int degree = (int)(angle / 100);
    double min = ((angle - degree * 100));

    deg = degree + ((double)min) / 60;

    return deg;
}

double DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::trans2Temperature(double adValue)
{
    //for new temperature sensor
    double tempM = 0;
    double k = adValue / FULLAD;
    double resValue = 0;
    if (k != 0)
    {
       resValue = 10000 * (1 - k) / k;      //AD value to resistor
    }
    else
    {
       resValue = resTable[0];
    }


    int index = -1;
    if (resValue >= resTable[0])       //too lower
    {
       tempM = -20;
    }
    else if (resValue <= resTable[24])
    {
       tempM = 100;
    }
    else
    {
       for (int i = 0; i < 24; i++)
       {
           if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1]))
           {
               index = i;
               break;
           }
       }
       if (index >= 0)
       {
           tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
       }
       else
       {
           tempM = 0;
       }

    }

    return tempM;
}



void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::handleComData(const char *data, const int nLen)
{
    char msgTail[] = {0x0d,0x0a};
    int nStartIndex, nUnProcessedPacketLen, nPacketIndex;
    char* unProcessedPacket = NULL;

    nStartIndex = 0;
    nPacketIndex = 0;

    if (_nMsgLen + nLen < MAXBUFLEN){
      memcpy(_dataBuf + _nMsgLen, data, nLen);
      _nMsgLen += nLen;
    }
    else{
      //clear the whole internal buffer, just keep the latest packet
      memcpy(_dataBuf,data,nLen);
      _nMsgLen = nLen;
    }

/*
    memcpy(_dataBuf,data,nLen);
    _nMsgLen = nLen;
*/
    while(nPacketIndex + 1 < _nMsgLen){
      // look for the MSG_TAIL
      if ( (_dataBuf[nPacketIndex] == msgTail[0]) && (nPacketIndex + 1 < _nMsgLen) && (_dataBuf[nPacketIndex + 1] == msgTail[1]))
   //  if ( (_dataBuf[nPacketIndex] == '\r') && (nPacketIndex + 1 < _nMsgLen) )
     {
        // find a data package
        unProcessedPacket = _dataBuf + nStartIndex;
	nPacketIndex += 2;
        nUnProcessedPacketLen = nPacketIndex - nStartIndex;
        nStartIndex = nPacketIndex;
        DealWithPacket(unProcessedPacket,nUnProcessedPacketLen);
        if (nPacketIndex == _nMsgLen){
          // the while data msg is processed, empty the buffer
          _nMsgLen = 0;
          return;
        }
        else{
	   
        }
        
      }
      else{
        nPacketIndex++;
      }

    }
    if (nPacketIndex >= _nMsgLen -1){
      // did not find a tail
      memcpy(_dataBuf,_dataBuf + nStartIndex,_nMsgLen - nStartIndex);
      _nMsgLen = _nMsgLen - nStartIndex;

    }
    return;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::debugCommMessage(std::string msg)
{

#ifdef DEBUG_ERROR
  printf("DrRobot Motion Sensor Driver: %s",msg.c_str());
#endif
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: getDrRobotMotionDriverConfig(DrRobotMotionConfig* driverConfig)
{
  strcpy(driverConfig->robotID,_robotConfig->robotID);
  driverConfig->robotType = _robotConfig->robotType;
  driverConfig->commMethod = _robotConfig->commMethod;
  driverConfig->portNum = _robotConfig->portNum;
  strcpy(driverConfig->robotIP, _robotConfig->robotIP);
  strcpy(driverConfig->serialPortName, _robotConfig->serialPortName);
  return;
}

void DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: setDrRobotMotionDriverConfig(DrRobotMotionConfig* driverConfig)
{

  strcpy(_robotConfig->robotID,driverConfig->robotID);
  _robotConfig->robotType = driverConfig->robotType;
  _robotConfig->commMethod = driverConfig->commMethod;
  _robotConfig->portNum = driverConfig->portNum;
  strcpy(_robotConfig->robotIP,driverConfig->robotIP);
  strcpy(_robotConfig->serialPortName,driverConfig->serialPortName);

 
  return;
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::readMotorSensorData(MotorSensorData* motorSensorData)
{
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(motorSensorData,&_motorSensorData,sizeof(MotorSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: readGPSSensorData(GPSSensorData* gpsSensorData)
{
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(gpsSensorData,&_gpsSensorData,sizeof(GPSSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
}


int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: readIMUSensorData(IMUSensorData* imuSensorData)
{

  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(imuSensorData, &_imuSensorData, sizeof(IMUSensorData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
 }

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver:: readMotorBoardData(MotorBoardData* motorBoardData)
{
  pthread_mutex_lock(&_mutex_Data_Buf);
  memcpy(motorBoardData, &_motorBoardData, sizeof(MotorBoardData));
  pthread_mutex_unlock(&_mutex_Data_Buf);
  return 0;
}

int DrRobot_MotionSensorDriver::DrRobotMotionSensorDriver::sendCommand(const char* msg, const int nLen)
{
  ssize_t retval = 0;
  char endTail[] = "\r\n";
  if (!_stopComm)
  {

    if ( (_robotConfig->commMethod == Network) && ( _sockfd > 0))
    {
      //int retval = sendto(_sockfd, msg, nLen, 0,(const struct sockaddr *)&_addr,sizeof(_addr));	//UDP
      int retval = send(_sockfd, msg, nLen, MSG_DONTWAIT);
          retval = send(_sockfd, endTail, 2, MSG_DONTWAIT);
      if (retval > 0)
      {
         return retval;
       }
       else
       {
         perror("Socket send");
         return -1;
       }
    }
    else if( (_robotConfig->commMethod == Serial) && (_serialfd > 0))
    {
      int origflags = fcntl(_serialfd, F_GETFL,0);
      fcntl(_serialfd,F_SETFL,origflags & ~O_NONBLOCK);

      retval = write(_serialfd, msg, nLen);
      int fputserrno = errno;
      fcntl(_serialfd,F_SETFL,origflags | O_NONBLOCK);
      errno =fputserrno;
      if (retval != -1)
      {
        return retval;
      }
      else
      {
        return -1;
      }

    }
  }
  return -1;
}



