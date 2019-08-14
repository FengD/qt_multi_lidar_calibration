#include "lidar_driver/rawdata.h"

namespace lidar_driver {
  RawDataP40P::RawDataP40P() : RawData() {

  }

  RawDataP40P::~RawDataP40P() {

  }

  int RawDataP40P::unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data) {
    packetCounter++;
    for(int i = 0; i < PACKET_BLOCKS; i++) {
        blockCalculate(cloud, data, i);
    }
    if(packetCounter % packetsCloud == 0) {
      packetCounter = 0;
      return 1;
    } else {
      return 0;
    }
  }

  int RawDataP40P::getPointCloudTimestamp(uint8_t *data) {
    int timestamp = ((data[1250]) + (data[1251]<<8) + (data[1252]<<16) + (data[1253]<<24)) ;
    return timestamp;
  }

  bool RawDataP40P::readCorrectionFile(std::string path) {
    boost::filesystem::path filePath(path);
    if (!boost::filesystem::is_regular(filePath)) {
      printf("invalid lidar correction file, use default values\n");
      return false;
    }

    std::ifstream ifs(filePath.string());
    if (!ifs.is_open())
      return false;

    std::string line;
    if (std::getline(ifs, line)) { // first line "Laser id,Elevation,Azimuth"
        std::cout << "parsing correction file." << std::endl;
    }

    int lineCounter = 0;
    while (std::getline(ifs, line)) {
      if (lineCounter++ >= 40)
          break;

      int lineId = 0;
      double elev, azimuth;

      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> lineId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> elev;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> azimuth;
      lineId--;
      lidarLineAngle[lineId] = elev;
      horizonCorrectAngle[lineId] = azimuth;
    }
    return true;
  }

  void RawDataP40P::setMode(int mode) {
    if(mode == 0) {
      packetsCloud = CLOUD_PACKETS;
    } else if(mode == 1) {
      packetsCloud = CLOUD_PACKETS_DUAL;
    }
  }

  void RawDataP40P::blockCalculate(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter) {
    // calculate azimuth (angle) by udp data byte
    float azimuth = (data[blockCounter * BYTE_PAR_BLOCK + 2] + (data[blockCounter * BYTE_PAR_BLOCK + 3] << 8)) / 100.0;
    // std::cout << azimuth << std::endl;
    pcl::PointXYZI p;
    for(int j = 0; j < CHANNELS_NUM; j++){
      channelCalculate(p, data, j, blockCounter, azimuth, 4);
      if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z)) {
        cloud.push_back(p);
      }
    }
  }

  void RawDataP40P::channelCalculate(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte) {
    // add correct value for horizontal angle and transform to arc
    float angle_horizontal = (azimuth + horizonCorrectAngle[channelCounter]) * M_PI / 180.0;
    // calculate sinus
    float angle_horizontal_sin = sin(angle_horizontal);
    // calculate cosus
    float angle_horizontal_cos = sqrt(1 - angle_horizontal_sin * angle_horizontal_sin);
    if (angle_horizontal > M_PI / 2 && angle_horizontal < 3 * M_PI / 2) {
        angle_horizontal_cos = -angle_horizontal_cos;
    }
    // the index of each start byte position of each channel in one block
    int index = startByte + channelCounter * 3 + blockCounter * BYTE_PAR_BLOCK;
    // decode the distance value from udp
    float distance = (data[index] + (data[index + 1] << 8)) * 0.004;
    // decode the intensity value from udp
    float intensity = (data[index + 2]);
    // distance project on xy plane
    float distance2xyPlane = 0.0;
    // distance project on z
    float distance2z = 0.0;
    // cos and sin of the offset angle
    float angle_vertical_cos = cos(lidarLineAngle[channelCounter] * M_PI / 180.0);
    float angle_vertical_sin = sin(lidarLineAngle[channelCounter] * M_PI / 180.0);

    distance2xyPlane = distance * angle_vertical_cos;
    distance2z = distance * angle_vertical_sin;
    // the index of the points
    // int point_index = (blockCounter * CHANNELS_NUM) + channelCounter;
    // calculate the x y z of one point
    float x_lidar_coordinate = distance2xyPlane * angle_horizontal_sin;
    float y_lidar_coordinate = distance2xyPlane * angle_horizontal_cos;
    float z_lidar_coordinate = distance2z;
    // float y_lidar_coordinate = cos(OFFSET_A) * distance2xyPlane * angle_horizontal_cos + sin(OFFSET_A) * distance2z;
    // float z_lidar_coordinate = -sin(OFFSET_A) * distance2xyPlane + cos(OFFSET_A) * distance2z;
    point.x = -y_lidar_coordinate;
    point.y = x_lidar_coordinate;
    point.z = z_lidar_coordinate;
    point.intensity = intensity;

    if (point.x == 0 && point.y == 0 && point.z == 0) {
      point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  RawDataVlp16::RawDataVlp16() : RawData() {

  }

  RawDataVlp16::~RawDataVlp16() {

  }

  int RawDataVlp16::unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data) {
    packetCounter++;
    for(int i = 0; i < PACKET_BLOCKS; i++) {
        blockCalculate(cloud, data, i);
    }
    if(packetCounter % packetsCloud == 0) {
      packetCounter = 0;
      return 1;
    } else {
      return 0;
    }
  }

  int RawDataVlp16::getPointCloudTimestamp(uint8_t *data) {
    int timestamp = ((data[1200]) + (data[1201]<<8) + (data[1202]<<16) + (data[1203]<<24)) ;
    return timestamp;
  }

  bool RawDataVlp16::readCorrectionFile(std::string path) {
    (void)path;
    std::cout << "no need for vlp16" << std::endl;
    return true;
  }

  void RawDataVlp16::setMode(int mode) {
    if(mode == 0) {
      packetsCloud = CLOUD_PACKETS;
    } else if(mode == 1) {
      packetsCloud = CLOUD_PACKETS_DUAL;
    }
  }

  void RawDataVlp16::blockCalculate(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter) {
    // calculate azimuth (angle) by udp data byte
    float azimuth = (data[blockCounter * BYTE_PAR_BLOCK + 2] + (data[blockCounter * BYTE_PAR_BLOCK + 3] << 8)) / 100.0;
    // std::cout << azimuth << std::endl;
    pcl::PointXYZI p;
    for(int j = 0; j < CHANNELS_NUM; j++){
      channelCalculate(p, data, j, blockCounter, azimuth, 4);
      if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z)) {
        cloud.push_back(p);
      }

      channelCalculate(p, data, j, blockCounter, (azimuth + 0.2), 52);
      if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z)) {
        cloud.push_back(p);
      }
    }
  }

  void RawDataVlp16::channelCalculate(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte) {
    // add correct value for horizontal angle and transform to arc
    float angle_horizontal = azimuth * M_PI / 180.0;
    // calculate sinus
    float angle_horizontal_sin = sin(angle_horizontal);
    // calculate cosus
    float angle_horizontal_cos = sqrt(1 - angle_horizontal_sin * angle_horizontal_sin);
    if (angle_horizontal > M_PI / 2 && angle_horizontal < 3 * M_PI / 2) {
        angle_horizontal_cos = -angle_horizontal_cos;
    }
    // the index of each start byte position of each channel in one block
    int index = startByte + channelCounter * 3 + blockCounter * BYTE_PAR_BLOCK;
    // decode the distance value from udp
    float distance = (data[index] + (data[index + 1] << 8)) * 0.002;
    std::cout << distance << std::endl;
    // decode the intensity value from udp
    float intensity = (data[index + 2]);
    // distance project on xy plane
    float distance2xyPlane = 0.0;
    // distance project on z
    float distance2z = 0.0;
    // cos and sin of the offset angle
    float angle_vertical_cos = cos(lidarLineAngle[channelCounter] * M_PI / 180.0);
    float angle_vertical_sin = sin(lidarLineAngle[channelCounter] * M_PI / 180.0);

    distance2xyPlane = distance * angle_vertical_cos;
    distance2z = distance * angle_vertical_sin;
    // the index of the points
    // int point_index = (blockCounter * CHANNELS_NUM) + channelCounter;
    // calculate the x y z of one point
    float x_lidar_coordinate = distance2xyPlane * angle_horizontal_sin;
    float y_lidar_coordinate = distance2xyPlane * angle_horizontal_cos;
    float z_lidar_coordinate = distance2z;
    // float y_lidar_coordinate = cos(OFFSET_A) * distance2xyPlane * angle_horizontal_cos + sin(OFFSET_A) * distance2z;
    // float z_lidar_coordinate = -sin(OFFSET_A) * distance2xyPlane + cos(OFFSET_A) * distance2z;
    point.x = y_lidar_coordinate;
    point.y = -x_lidar_coordinate;
    point.z = z_lidar_coordinate;
    point.intensity = intensity;

    if (point.x == 0 && point.y == 0 && point.z == 0) {
      point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    }
  }
}
