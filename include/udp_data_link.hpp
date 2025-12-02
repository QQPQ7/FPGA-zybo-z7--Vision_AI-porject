#pragma once
#include <nlohmann/json.hpp>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <poll.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fstream>

class UDPDataLink {
public:
    using DataReceivedHandler = std::function<void(const std::vector<uint8_t>&, const std::string&, uint16_t)>;

    int LocalPort{5000};
    std::string RemoteIP{"192.168.50.50"};  //수작업 변경시 {"192.168.137.1"}; 
    int RemotePort{5000};

    DataReceivedHandler DataReceived;

    explicit UDPDataLink(int local = 5000){
        LocalPort = local;
        load_config("appsettings.json");
    }

    bool start(){
        if (sock_ != -1) return true;
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) { perror("socket"); return false; }

        sockaddr_in addr{}; addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(LocalPort);
        if (::bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("bind"); ::close(sock_); sock_ = -1; return false;
        }

        running_.store(true);
        rx_thread_ = std::thread([this]{ this->rx_loop(); });
        return true;
    }

    void stop(){
        running_.store(false);
        if (sock_ != -1) {
            ::shutdown(sock_, SHUT_RDWR);
            ::close(sock_);
            sock_ = -1;
        }
        if (rx_thread_.joinable()) rx_thread_.join();
    }

    bool send_text(const std::string& s){
        return send_bytes(std::vector<uint8_t>(s.begin(), s.end()));
    }

    bool send_bytes(const std::vector<uint8_t>& data){
        if (sock_ == -1) return false;
        sockaddr_in to{}; to.sin_family = AF_INET;
        to.sin_port = htons(RemotePort);
        if (::inet_pton(AF_INET, RemoteIP.c_str(), &to.sin_addr) != 1) {
            std::fprintf(stderr, "[UDP] invalid RemoteIP: %s\n", RemoteIP.c_str());
            return false;
        }
        ssize_t n = ::sendto(sock_, data.data(), data.size(), 0, (sockaddr*)&to, sizeof(to));
        return (n == (ssize_t)data.size());
    }

    ~UDPDataLink(){ stop(); }

private:
    int sock_{-1};
    std::thread rx_thread_;
    std::atomic<bool> running_{false};

    void rx_loop(){
        std::vector<uint8_t> buf(64*1024);
        while (running_.load()){
            pollfd pfd{sock_, POLLIN, 0};
            int pr = ::poll(&pfd, 1, 500);
            if (pr <= 0) continue;
            sockaddr_in from{}; socklen_t flen = sizeof(from);
            ssize_t n = ::recvfrom(sock_, buf.data(), buf.size(), 0, (sockaddr*)&from, &flen);
            if (n <= 0) continue;

            char ipstr[INET_ADDRSTRLEN]; ::inet_ntop(AF_INET, &from.sin_addr, ipstr, sizeof(ipstr));
            uint16_t port = ntohs(from.sin_port);
            if (DataReceived) DataReceived(std::vector<uint8_t>(buf.begin(), buf.begin()+n), ipstr, port);
        }
    }

    void load_config(const std::string& path){
        std::ifstream ifs(path);
        if (!ifs.is_open()) return;
        try{
            nlohmann::json j; ifs >> j;
            auto n = j.at("NetworkConfig");
            if (n.contains("RemoteIP"))   RemoteIP  = n.at("RemoteIP").get<std::string>();
            if (n.contains("RemotePort")) RemotePort= n.at("RemotePort").get<int>();
            if (n.contains("LocalPort"))  LocalPort = n.at("LocalPort").get<int>();
        } catch(...){
            std::fprintf(stderr, "[UDP] config parse ignored\n");
        }
    }
};
