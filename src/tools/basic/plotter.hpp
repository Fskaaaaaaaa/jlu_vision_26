#pragma once

#include <netinet/in.h> // sockaddr_in
#include <nlohmann/json.hpp>

#include <mutex>
#include <string>

namespace tools {
class Plotter {
public:
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);
  ~Plotter();
  void plot(const nlohmann::json &json);
  void plot(const std::string &key, double value);

private:
  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;
};

} // namespace tools
