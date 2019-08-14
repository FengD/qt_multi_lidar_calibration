/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 */

#ifndef _INPUT_H_
#define _INPUT_H_

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

namespace lidar_driver {
  // default data port
  static uint16_t DATA_PORT_NUMBER = 2368;

  class Input {
  public:
    /**
    * constructor
    * @param {string} deviceIp
    * @param {int} port
    */
    Input(std::string deviceIp, uint16_t port = DATA_PORT_NUMBER);

    /**
    * destructor
    */
    ~Input();

    /*
    * @brief get the udp packets by given size
    * @param {pointer} get the packets and put in the array
    * @param {int} size of the packet
    */
    int getPacket(uint8_t *pkt, int packet_size);

    /**
    * @brief set the device
    * @param {string} device ip
    */
    void setDeviceIP( const std::string& ip );

  private:

  private:
    int sockfd_;
    in_addr devip_;
    std::string deviceIp_;
  };

} // lidar_driver namespace

#endif // _INPUT_H_
