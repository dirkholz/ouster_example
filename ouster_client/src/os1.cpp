#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "ouster/os1.h"

namespace ouster {
namespace OS1 {

using ns = std::chrono::nanoseconds;

struct client {
    int lidar_fd;
    int imu_fd;
    std::string beam_intrinsics;
    std::string lidar_intrinsics;
    std::string imu_intrinsics;
    std::string sensor_info;
    ~client() {
        close(lidar_fd);
        close(imu_fd);
    }
};

static int udp_data_socket(int port) {
    int sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        std::cerr << "socket: " << std::strerror(errno) << std::endl;
        return -1;
    }

    struct sockaddr_in my_addr;
    memset((char*)&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(port);
    my_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_fd, (struct sockaddr*)&my_addr, sizeof(my_addr)) < 0) {
        std::cerr << "bind: " << std::strerror(errno) << std::endl;
        return -1;
    }

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                   sizeof(timeout)) < 0) {
        std::cerr << "setsockopt: " << std::strerror(errno) << std::endl;
        return -1;
    }

    return sock_fd;
}

static int cfg_socket(const char* addr) {
    struct addrinfo hints, *info_start, *ai;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int ret = getaddrinfo(addr, "7501", &hints, &info_start);
    if (ret != 0) {
        std::cerr << "getaddrinfo: " << gai_strerror(ret) << std::endl;
        return -1;
    }
    if (info_start == NULL) {
        std::cerr << "getaddrinfo: empty result" << std::endl;
        return -1;
    }

    int sock_fd;
    for (ai = info_start; ai != NULL; ai = ai->ai_next) {
        sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock_fd < 0) {
            std::cerr << "socket: " << std::strerror(errno) << std::endl;
            continue;
        }

        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        unsigned long mode = 1;
        int result;
        if ((result = ioctl(sock_fd, FIONBIO, &mode)) < 0) {
          std::cout << "ioctlsocket failed with error: " << result << std::endl;
        }

        if ((result = connect(sock_fd, ai->ai_addr, ai->ai_addrlen)) == -1) {
          if (errno != EINPROGRESS) {
            std::cout << "connect failed with error: " << result << std::endl;
            close(sock_fd);
            continue;
          }
        }

        if (result == 0) {
          return sock_fd;
        }

        fd_set rset, wset;
        FD_ZERO(&rset);
        FD_SET(sock_fd, &rset);
        wset = rset;
        if ((result = select(sock_fd + 1, &rset, &wset, NULL, &timeout)) < 0)
          return -1;
        if (result == 0) {
          errno = ETIMEDOUT;
          return -1;
        }

        mode = 0;
        if ((result = ioctl(sock_fd, FIONBIO, &mode)) < 0) {
          std::cout << "ioctlsocket failed with error: " << result << std::endl;
        }

        fd_set write, error;
        FD_ZERO(&write);
        FD_ZERO(&error);
        FD_SET(sock_fd, &write);
        FD_SET(sock_fd, &error);

        select(0, NULL, &write, &error, &timeout);
        if (FD_ISSET(sock_fd, &write)) {
          break;
        }

        continue;
    }

    freeaddrinfo(info_start);
    if (ai == NULL) {
        return -1;
    }

    return sock_fd;
}

std::shared_ptr<client> init_client(
    const std::string& hostname, const std::string& udp_dest_host,
    int lidar_port, int imu_port,
    const std::vector<std::pair<std::string, std::string>>& config_commands,
    const bool print_debug_output) {
    int sock_fd = cfg_socket(hostname.c_str());

    if (sock_fd < 0) return std::shared_ptr<client>();

    std::string cmd;
    std::string beam_intrinsics;
    std::string lidar_intrinsics;
    std::string imu_intrinsics;
    std::string sensor_info;

    auto do_cmd = [&](const std::string& op, const std::string& val) {
        const size_t max_res_len = 4096;
        char read_buf[max_res_len + 1];

        if (print_debug_output) {
            std::cout << "OUSTER: Sending command \""
                      << op << "\" with value \""
                      << val << "\"." << std::endl;
        }

        ssize_t len;
        std::string cmd = op + " " + val + "\n";
        len = write(sock_fd, cmd.c_str(), cmd.length());
        if (len != (ssize_t)cmd.length()) {
            std::cerr << "OUSTER: failed to send command" << std::endl;
            return false;
        }

        len = read(sock_fd, read_buf, max_res_len);
        if (len < 0) {
            std::cerr << "OUSTER read error: " << std::strerror(errno) << std::endl;
            return false;
        }
        read_buf[len] = '\0';

        auto res = std::string(read_buf);
        res.erase(res.find_last_not_of(" \r\n\t") + 1);

        if (print_debug_output) {
            std::cout << "OUSTER: Received response \""
                      << res << "\"." << std::endl;
        }

        if (op == "get_beam_intrinsics") {
            beam_intrinsics = res;
        } else if (op == "get_lidar_intrinsics") {
            lidar_intrinsics = res;
        } else if (op == "get_imu_intrinsics") {
            imu_intrinsics = res;
        } else if (op == "get_sensor_info") {
            sensor_info = res;
        } else if (res != op) {
            std::cerr << "OUSTER: command \""
                      << op << "\" failed with \""
                      << res << "\"" << std::endl;
            return false;
        }
        return true;
    };

    bool success = true;
    success &= do_cmd("set_udp_port_lidar", std::to_string(lidar_port));
    success &= do_cmd("set_udp_port_imu", std::to_string(imu_port));
    success &= do_cmd("set_udp_ip", udp_dest_host);
    if (!config_commands.empty()) {
        for (const auto& config : config_commands) {
            success &= do_cmd(config.first, config.second);
        }
    }

    success &= do_cmd("get_beam_intrinsics", "");
    success &= do_cmd("get_lidar_intrinsics", "");
    success &= do_cmd("get_imu_intrinsics", "");
    success &= do_cmd("get_sensor_info", "");

    if (!success) return std::shared_ptr<client>();

    close(sock_fd);

    int lidar_fd = udp_data_socket(lidar_port);
    int imu_fd = udp_data_socket(imu_port);
    auto cli = std::make_shared<client>();
    cli->lidar_fd = lidar_fd;
    cli->imu_fd = imu_fd;
    cli->beam_intrinsics = beam_intrinsics;
    cli->lidar_intrinsics = lidar_intrinsics;
    cli->imu_intrinsics = imu_intrinsics;
    cli->sensor_info = sensor_info;
    return cli;
}

client_state poll_client(const client& c) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(c.lidar_fd, &rfds);
    FD_SET(c.imu_fd, &rfds);

    int max_fd = std::max(c.lidar_fd, c.imu_fd);

    int retval = select(max_fd + 1, &rfds, NULL, NULL, NULL);

    client_state res = client_state(0);
    if (retval == -1) {
        std::cerr << "select: " << std::strerror(errno) << std::endl;
        res = client_state(res | ERROR);
    } else if (retval) {
        if (FD_ISSET(c.lidar_fd, &rfds)) res = client_state(res | LIDAR_DATA);
        if (FD_ISSET(c.imu_fd, &rfds)) res = client_state(res | IMU_DATA);
    }
    return res;
}

static bool recv_fixed(int fd, void* buf, size_t len) {
    ssize_t n = recvfrom(fd, buf, len + 1, 0, NULL, NULL);
    if (n == (ssize_t)len)
        return true;
    else if (n == -1)
        std::cerr << "recvfrom: " << std::strerror(errno) << std::endl;
    else
        std::cerr << "Unexpected udp packet length: " << n << std::endl;
    return false;
}

bool read_lidar_packet(const client& cli, uint8_t* buf) {
    return recv_fixed(cli.lidar_fd, buf, lidar_packet_bytes);
}

bool read_imu_packet(const client& cli, uint8_t* buf) {
    return recv_fixed(cli.imu_fd, buf, imu_packet_bytes);
}

std::string get_beam_intrinsics(const client& cli) {
  return cli.beam_intrinsics;
}

std::string get_lidar_intrinsics(const client& cli) {
  return cli.lidar_intrinsics;
}

std::string get_imu_intrinsics(const client& cli) {
  return cli.imu_intrinsics;
}

std::string get_sensor_info(const client& cli) {
  return cli.sensor_info;
}

}
}
