#ifndef _SIMPLE_UDP_HPP_
#define _SIMPLE_UDP_HPP_
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <stdlib.h>

namespace udp_class {

struct packageHeader{
  char senderIP[15];
  char receiverIP[15];
  uint16_t receiverPort;
  uint16_t packageLength;
  uint16_t packageKind;
};

struct Box
{
    float x;
    float y;
    float z;
    float l;
    float w;
    float h;
    float yaw;
    float score;
    int label;
};

struct BoxList
{
  double timeStamp;
  uint16_t boxNum;
  Box box_list[50];
};

struct BoxData
{
  packageHeader header;
  BoxList obj_boxes;
};

struct LidarObject
{
  float objCornerPoint[12];
  float objCenterPoint[3];
  float objNearestPoint[3];
  float objVelocity[3];
  float objDimension[3];
  float objLifeTime;
  float objConfidence;
  float objOrietation;
  uint16_t objId;
  //UNKNOWN = 0,UNKNOWN_MOVABLE = 1,UNKNOWN_UNMOVABLE = 2,
  //PEDESTRIAN = 3,BICYCLE = 4,VEHICLE = 5,MAX_OBJECT_TYPE = 6,
  uint16_t objType;
  bool isInMap;
};

struct ObjectList
{
  uint64_t timeStamp;
  uint16_t objNum;
  float objProcessTime;
  LidarObject objList[50];
};

struct ObjectData
{
  packageHeader header;
  ObjectList objects;
};

class UdpClient
{
public:
  UdpClient(std::string ip_ = "127.0.0.1", int port_ = 8731);
  ~UdpClient(){
    close(m_sockSend);
  }
  void SendMessage(const ObjectData &objData);

private:
  int m_sockSend;
  struct sockaddr_in clientAddr;

};

class UdpServer
{
public:
  UdpServer(int port_ = 7431);
  ~UdpServer(){
    close(m_sockRecv);
  }
  void RecvMessage(BoxData &box_data);

private:
  int m_sockRecv;
  struct sockaddr_in serverAddr;
  struct sockaddr_in clientAddr;
};

}

#endif //_SIMPLE_UDP_HPP_
