/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 */

/**
 * class lidar driver
 *
 */

#include <unistd.h>
#include <string>
#include <string.h>
#include <sstream>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include "lidar_driver/input.h"

namespace lidar_driver {
  ////////////////////////////////////////////////////////////////////////
  // Input class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   */
  Input::Input(std::string deviceIp, uint16_t port) {
    sockfd_ = -1;
    deviceIp_ = "";
    if (!deviceIp.empty()) {
      inet_aton(deviceIp.c_str(), &devip_);
      deviceIp_ = deviceIp;
    }

    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1) {
      perror("socket"); // Socket Error
      return;
    }

    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(port);          // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP



    // used for multipule lidar
    int opt = 1;
    setsockopt( sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt) );

    if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
      perror("bind"); // Bind Error
      return;
    }

    if (fcntl(sockfd_,F_SETFL, O_NONBLOCK|FASYNC) < 0) {
      perror("non-block");
      return;
    }

    // struct ip_mreq mreq;
    // mreq.imr_multiaddr.s_addr = inet_addr("224.0.10.0");
    // mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    // if (
    //     setsockopt(
    //         sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)
    //     ) < 0
    // ){
    //     perror("setsockopt");
    //     return;
    // }
  }

  /** @brief destructor */
  Input::~Input(void) {
    (void) close(sockfd_);
  }

  /** @brief Get one velodyne packet. */
  int Input::getPacket(uint8_t *pkt, int packet_size) {
    struct pollfd fds[1];
    fds[0].fd = sockfd_;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true) {
      do {
        int retval = poll(fds, 1, POLL_TIMEOUT);
        if (retval < 0)             // poll() error?
          {
            if (errno != EINTR)
              std::cerr << "poll() error: " << errno << std::endl;
            return -1;
          }
        if (retval == 0)            // poll() timeout?
          {
            std::cout << "poll() timeout" << std::endl;
            return -1;
          }
        if ((fds[0].revents & POLLERR)
            || (fds[0].revents & POLLHUP)
            || (fds[0].revents & POLLNVAL)) // device error?
          {
            std::cout << "poll() reports Velodyne error" << std::endl;
            return -1;
          }
      } while ((fds[0].revents & POLLIN) == 0);

      // Receive packets that should now be available from the
      // socket using a blocking read.
      ssize_t nbytes = recvfrom(sockfd_, &pkt[0],
                                packet_size,  0,
                                (sockaddr*) &sender_address,
                                &sender_address_len);

      if (nbytes < 0) {
        if (errno != EWOULDBLOCK) {
          perror("recvfail");
          return -1;
        }
      } else if ((size_t) nbytes == (size_t)packet_size) {
        // read successful,
        // if packet is not from the lidar scanner we selected by IP,
        // continue otherwise we are done
        if(deviceIp_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
          continue;
        else
          break; //done
      }
    }
    return 0;
  }

} // lidar_driver namespace
