// Is the planner process that sent the mode's commands still there? (owner decision 2026-10-29)
//
// Each command's sender (its connection header's callerid) is remembered per mode; a thread asks that
// node for its pid over its XML-RPC slave API (getPid, as `rosnode ping` does) every `period` s.
//   alive(node) -> true   it answered within the last `hung_s`
//                  false  gone: connection refused or not registered with the master (a killed or exited
//                         process: decisive at once), or no answer for `hung_s` (hung)
//                  none   unknown (nothing known yet, the master unreachable, first answers pending)
// The silence of commands alone cannot tell a dead planner from a live one with nothing to send (SUPER
// with no plan, the track planner at NO_PATH or REACHED), whose hold stays resumable.
#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace l4_bridge {

// one XML-RPC call with string arguments over HTTP, each socket operation limited to timeout_s (Python's
// xmlrpc.client.ServerProxy with a socket timeout)
struct RpcResult {
  enum Status { OK, TIMEOUT, ERROR } status = ERROR;   // ERROR: refused, unreachable, HTTP error, a fault
  int code = 0;                                        // the ROS API's [code, message, value]
  std::string value;                                   // the value as text (a string or an int)
  std::string error;
};
RpcResult xmlrpc_call(const std::string& uri, const std::string& method, const std::vector<std::string>& args,
                      double timeout_s);

class PlannerLiveness {
 public:
  PlannerLiveness(std::string caller, double period = 0.1, double timeout = 0.25, double hung_s = 1.0);
  ~PlannerLiveness();
  void start();   // the ping thread (needs ros::init)

  std::optional<bool> alive(const std::optional<std::string>& node) const;
  void set_node(const std::string& mode, const std::string& node);   // the sender of a mode's command
  std::optional<std::string> node(const std::string& mode) const;
  void want(const std::optional<std::string>& node);                  // the node to watch

 private:
  void loop();
  void ping(const std::string& node);
  void set(const std::string& node, std::optional<bool> alive, bool answered = false);
  // master.lookupNode: OK with the URI, ERROR with code -1 (not registered), or no verdict (no master)
  RpcResult lookup(const std::string& node);

  std::string caller_;
  double period_, timeout_, hung_s_;
  std::map<std::string, std::string> node_;   // mode -> the node that sent its latest command
  std::optional<std::string> want_;
  std::map<std::string, std::string> uri_;    // ping thread only
  std::map<std::string, std::pair<std::optional<bool>, std::optional<double>>> verdict_;   // node -> (alive, last answer)
  mutable std::mutex lock_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace l4_bridge
