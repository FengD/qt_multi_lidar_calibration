/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 */

#ifndef _RAW_DATA_H_
#define _RAW_DATA_H_

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <unistd.h>

namespace lidar_driver {
  /**
  * interface of the lidar data processing class
  */
  class RawData {
  public:
    RawData() {}

    virtual ~RawData() {}

    virtual int unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data) = 0;

    virtual int getPointCloudTimestamp(uint8_t *data) = 0;

    virtual bool readCorrectionFile(std::string path) = 0;

    virtual void setMode(int mode) = 0;

  protected:
    // count if the packet is the last packet of one cloud
    int packetCounter = 0;
    // the packets number of one pointcloud
    int packetsCloud = 0;
  private:

    virtual void blockCalculate(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter) = 0;

    virtual void channelCalculate(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte) = 0;

  };

  /**
  * hesai pandar 40
  */
  class RawDataP40P : public RawData {
  public:
    RawDataP40P();
    ~RawDataP40P();

    int unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data);

    int getPointCloudTimestamp(uint8_t *data);

    bool readCorrectionFile(std::string path);

    void setMode(int mode);

  private:
    // defined by the User Guide
    static const int DATA_SIZE = 1240;
    static const int BYTE_PAR_BLOCK = 124;
    static const int CLOUD_PACKETS = 180; // single return mode by default
    static const int CLOUD_PACKETS_DUAL = 2 * CLOUD_PACKETS;
    static const int PACKET_BLOCKS = 10;
    static const int CHANNELS_NUM = 40;
    static const int POINTS_NUM = (PACKET_BLOCKS * CHANNELS_NUM);

    // default vertical angle
    float lidarLineAngle[CHANNELS_NUM] = {
        7.00, 6.00, 5.00, 4.00, 3.00, 2.00,
        1.67, 1.33, 1.00, 0.67, 0.33, 0.00, -0.33, -0.67,
        -1.00, -1.33, -1.67, -2.00, -2.33, -2.67, -3.00, -3.33, -3.67,
        -4.00, -4.33, -4.67, -5.00, -5.33, -5.67,
        -6.00, -7.00, -8.00, -9.00, -10.0, -11.0, -12.0, -13.0, -14.0, -15.0, -16.0};

    // default horizontal angle correction
    float horizonCorrectAngle[CHANNELS_NUM] = {
        0.0, 0.0, 0.0, 0.0, -2.5, -2.5, 2.5, -5.0,
        -2.5, 2.5, -5.0, 2.5, 2.5, -5.0, 0.0, 2.5,
        -5.0, 0.0, 5.0, -2.5, 0.0, 5.0, -2.5, 0.0,
        5.0, -2.5, 2.5, 5.0, -2.5, 2.5, 2.5, 2.5,
        0.0, 0.0, 0.0, 0.0, -2.5, -2.5, -2.5, -2.5
    };

    void blockCalculate(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter);

    void channelCalculate(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte);

  };

  /**
  * velodyne vlp 16
  */
  class RawDataVlp16 : public RawData {
  public:
    RawDataVlp16();
    ~RawDataVlp16();

    int unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data);

    int getPointCloudTimestamp(uint8_t *data);

    bool readCorrectionFile(std::string path);

    void setMode(int mode);

  private:
    // defined by the User Guide
    static const int DATA_SIZE = 1200;
    static const int BYTE_PAR_BLOCK = 100;
    static const int CLOUD_PACKETS = 76; // single return mode by default
    static const int CLOUD_PACKETS_DUAL = 2 * CLOUD_PACKETS;
    static const int PACKET_BLOCKS = 12;
    static const int CHANNELS_NUM = 16;
    static const int POINTS_NUM = (2 * PACKET_BLOCKS * CHANNELS_NUM);

    float lidarLineAngle[CHANNELS_NUM] = {
      -15.00 ,1.00, -13.00, 3.00, -11.00, 5.00,
      -9.00, 7.00, -7.00, 9.00, -5.00, 11.00, -3.00, 13.00,
      -1.00, 15.00};

    void blockCalculate(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter);

    void channelCalculate(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte);

  };

}

#endif // _RAW_DATA_H_
