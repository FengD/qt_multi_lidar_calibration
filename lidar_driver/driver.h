/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 *  Description: The driver which used to receive the udp packets and process the packets to point cloud
 */

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <string>
#include <pthread.h>
#include <semaphore.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "lidar_driver/input.h"
#include "lidar_driver/rawdata.h"

#define UDPPACKETSIZE (1500)

namespace lidar_driver {
  // the udp packet struct
  struct Packet{
    uint8_t data[UDPPACKETSIZE];
  };

  class Driver {
  public:
    /**
    * constructor
    * @param {string} device ip address
    * @param {int} receive data port
    * @param {string} model of the lidar "like P40P, VLP16, etc."
    * @param {int} lidar mode {single return (0), dual return (1)}
    * @param {string} the path of the correction file, if it is empty the default value will be used
    * @param {methode} callback
    */
    Driver(std::string deviceIp, int dataPort, std::string model, int mode, std::string correctionfile, std::string deviceName, boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp, std::string deviceName)> lidarCallback);

    /**
    * destructor
    */
    virtual ~Driver();

    /**
    * @brief {start function used to start the threads
    */
    void start();

    /**
    * @brief {stop function used to stop the threads}
    */
    void stop();

  private:
    // the frameId of the point cloud, useful in ROS
    std::string frameId;
    // the size of the lidar udp packet except header, used in "recvfrom"
    int packetSize;
    // the lidar mark and serial like P40P VLP16
    std::string lidarModel;
    // the list of the udp packets
    std::list<Packet> lidarPacketList;
    // udp input object
    boost::shared_ptr<Input> input_;
    // udp process object
    boost::shared_ptr<RawData> rawData_;
    // the mutex lock of the lidar list
    pthread_mutex_t lidarLock_;
    // the semophore of the lidar packets receiver
    sem_t lidarSem_;
    // name of the lidar
    std::string deviceName_;
    // is the threads run
    bool continueLidarRecvThread;
    bool continueLidarProcessThread;
    // callback of the point cloud
    boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp, std::string deviceName)> userLidarCallback;
    // the two threads
    boost::thread *lidarRecvThread;
    boost::thread *lidarProcessThread;
    /*
    * @brief the function of the lidarRecvThread
    */
    void lidarPacketRecvFn();
    /*
    * @brief the function of the lidarProcessThread
    */
    void processLidarPacketFn();
    /*
    * @brief push the received packets in the list
    * @param {struct} udp packet structure
    */
    void pushLidarData(Packet pkt);

  };

} // namespace lidar_driver

#endif // _DRIVER_H_
