#include "lidar_driver/driver.h"

namespace lidar_driver {

  Driver::Driver(std::string deviceIp, int dataPort, std::string model, int mode, std::string correctionfile, std::string deviceName,
                 boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp, std::string deviceName)> lidarCallback) {
    // init the udp input
    input_.reset(new Input(deviceIp, dataPort));

    deviceName_ = deviceName;
    // init bool
    continueLidarRecvThread = false;
    continueLidarProcessThread = false;
    // check the lidar model
    // and init the different data processing raw data
    if(model == "VLP16") {
      packetSize = 1206;
      frameId = "velodyne";
      rawData_.reset(new RawDataVlp16());
    } else if(model == "P40P") {
      packetSize = 1256;
      frameId = "pandar";
      rawData_.reset(new RawDataP40P());
    }
    if(!rawData_->readCorrectionFile(correctionfile)){
      std::cout << "read correction file error" << std::endl;
    }
    // set the lidar mode(single / dual)
    rawData_->setMode(mode);
    // init threads
    lidarRecvThread = 0;
    lidarProcessThread = 0;
    // assignment callback
    userLidarCallback = lidarCallback;
    // init semophore
    sem_init(&lidarSem_, 0, 0);
    // init lock
    pthread_mutex_init(&lidarLock_, NULL);
  }

  Driver::~Driver() {
    stop();
  }

  void Driver::start() {
    stop();
    continueLidarRecvThread = true;
    continueLidarProcessThread = true;
    lidarRecvThread = new boost::thread(boost::bind(&Driver::lidarPacketRecvFn, this));
    lidarProcessThread = new boost::thread(boost::bind(&Driver::processLidarPacketFn, this));
  }

  void Driver::stop() {
    continueLidarRecvThread = false;
    continueLidarProcessThread = false;

    if (lidarProcessThread) {
        lidarProcessThread->join();
        delete lidarProcessThread;
        lidarProcessThread = 0;
    }
    if (lidarRecvThread) {
        lidarRecvThread->join();
        delete lidarRecvThread;
        lidarRecvThread = 0;
    }
  }

  void Driver::lidarPacketRecvFn() {
    Packet p;

    while(continueLidarRecvThread) {
      int rc = input_->getPacket(p.data, packetSize);
        if (rc == -1) {
            continue;
        }
        if (rc == 1) {
            continue;
        }
      // if a right and whole packets of udp has been received
        pushLidarData(p);
    }
  }

  void Driver::pushLidarData(Packet pkt) {
    pthread_mutex_lock(&lidarLock_);
    lidarPacketList.push_back(pkt);
    if (lidarPacketList.size() > 6) {
        sem_post(&lidarSem_);
    }
    pthread_mutex_unlock(&lidarLock_);
  }

  void Driver::processLidarPacketFn() {
    int lastTimestamp = 0;
    struct timespec ts;

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> outMsg(new pcl::PointCloud<pcl::PointXYZI>());
    while (continueLidarProcessThread) {

        if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
            std::cout << "get time error" << std::endl;
        }

        ts.tv_sec += 1;
        if (sem_timedwait(&lidarSem_, &ts) == -1) {
            continue;
        }
        pthread_mutex_lock(&lidarLock_);
      Packet packet = lidarPacketList.front();
      lidarPacketList.pop_front();
        pthread_mutex_unlock(&lidarLock_);

      int ret = rawData_->unpack(*outMsg, packet.data);
      lastTimestamp = rawData_->getPointCloudTimestamp(packet.data);
        outMsg->header.frame_id = frameId;
        outMsg->height = 1;
      if(ret == 1 && outMsg->points.size() > 0) {
        if (userLidarCallback) {
          userLidarCallback(outMsg, lastTimestamp, deviceName_);
        }
        outMsg->clear();
      }
    }
  }

}
