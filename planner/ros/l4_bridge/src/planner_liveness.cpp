// Planner liveness (see planner_liveness.h): port of PlannerLiveness in the former planner_bridge.py.
#include "l4_bridge/planner_liveness.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>

#include <ros/ros.h>

namespace l4_bridge {

namespace {

std::string xml_escape(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '&') o += "&amp;";
    else if (c == '<') o += "&lt;";
    else if (c == '>') o += "&gt;";
    else o += c;
  }
  return o;
}

std::string xml_unescape(std::string s) {
  const std::pair<const char*, const char*> rep[] = {{"&lt;", "<"}, {"&gt;", ">"}, {"&quot;", "\""}, {"&apos;", "'"},
                                                     {"&amp;", "&"}};
  for (const auto& r : rep) {
    size_t p = 0;
    while ((p = s.find(r.first, p)) != std::string::npos) {
      s.replace(p, std::strlen(r.first), r.second);
      p += std::strlen(r.second);
    }
  }
  return s;
}

std::string trim(const std::string& s) {
  size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
  return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}

// the scalar values of the response's top-level array: <value><int>1</int></value>, <value>text</value>, ...
std::vector<std::string> array_values(const std::string& body) {
  std::vector<std::string> out;
  size_t p = body.find("<data>");
  if (p == std::string::npos) return out;
  const size_t end = body.find("</data>", p);
  while (true) {
    size_t a = body.find("<value>", p);
    if (a == std::string::npos || (end != std::string::npos && a > end)) break;
    a += 7;
    const size_t b = body.find("</value>", a);
    if (b == std::string::npos) break;
    std::string v = trim(body.substr(a, b - a));
    if (!v.empty() && v[0] == '<') {
      // <type>content</type> or <type/>
      const size_t gt = v.find('>');
      if (gt != std::string::npos && v[gt - 1] == '/') {
        v.clear();
      } else {
        const size_t lt = v.rfind("</");
        v = (gt != std::string::npos && lt != std::string::npos && lt > gt) ? v.substr(gt + 1, lt - gt - 1) : "";
      }
    }
    out.push_back(xml_unescape(v));
    p = b + 8;
  }
  return out;
}

// wait for fd readiness; false on a timeout
bool wait_fd(int fd, short ev, double timeout_s) {
  pollfd pfd{fd, ev, 0};
  int ms = static_cast<int>(timeout_s * 1000.0 + 0.5);
  while (true) {
    int r = ::poll(&pfd, 1, ms);
    if (r < 0 && errno == EINTR) continue;
    return r > 0;
  }
}

}  // namespace

RpcResult xmlrpc_call(const std::string& uri, const std::string& method, const std::vector<std::string>& args,
                      double timeout_s) {
  RpcResult res;
  // http://host:port/path
  std::string rest = uri;
  if (rest.rfind("http://", 0) == 0) rest = rest.substr(7);
  const size_t slash = rest.find('/');
  std::string hostport = rest.substr(0, slash), path = slash == std::string::npos ? "/" : rest.substr(slash);
  const size_t colon = hostport.rfind(':');
  std::string host = hostport.substr(0, colon), port = colon == std::string::npos ? "80" : hostport.substr(colon + 1);

  addrinfo hints{}, *ai = nullptr;
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  if (::getaddrinfo(host.c_str(), port.c_str(), &hints, &ai) != 0 || !ai) {
    res.error = "cannot resolve " + host;
    return res;
  }
  int fd = -1;
  bool timed_out = false;
  for (addrinfo* p = ai; p; p = p->ai_next) {
    fd = ::socket(p->ai_family, p->ai_socktype | SOCK_CLOEXEC, p->ai_protocol);
    if (fd < 0) continue;
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
    int r = ::connect(fd, p->ai_addr, p->ai_addrlen);
    if (r == 0) break;
    if (errno == EINPROGRESS) {
      if (wait_fd(fd, POLLOUT, timeout_s)) {
        int err = 0;
        socklen_t len = sizeof err;
        ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err == 0) break;
        res.error = std::strerror(err);
      } else {
        timed_out = true;
      }
    } else {
      res.error = std::strerror(errno);
    }
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(ai);
  if (fd < 0) {
    res.status = timed_out ? RpcResult::TIMEOUT : RpcResult::ERROR;
    if (timed_out) res.error = "connect timed out";
    return res;
  }

  std::string body = "<?xml version=\"1.0\"?>\n<methodCall><methodName>" + xml_escape(method) + "</methodName><params>";
  for (const auto& a : args) body += "<param><value><string>" + xml_escape(a) + "</string></value></param>";
  body += "</params></methodCall>\n";
  std::string req = "POST " + path + " HTTP/1.1\r\nHost: " + hostport +
                    "\r\nUser-Agent: l4_bridge\r\nContent-Type: text/xml\r\nContent-Length: " + std::to_string(body.size()) +
                    "\r\nConnection: close\r\n\r\n" + body;
  size_t sent = 0;
  while (sent < req.size()) {
    if (!wait_fd(fd, POLLOUT, timeout_s)) {
      ::close(fd);
      res.status = RpcResult::TIMEOUT;
      res.error = "send timed out";
      return res;
    }
    ssize_t n = ::send(fd, req.data() + sent, req.size() - sent, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      res.error = std::strerror(errno);
      ::close(fd);
      return res;
    }
    sent += static_cast<size_t>(n);
  }

  std::string resp;
  long content_length = -1;
  size_t header_end = std::string::npos;
  char buf[4096];
  while (true) {
    if (header_end != std::string::npos && content_length >= 0 &&
        resp.size() >= header_end + 4 + static_cast<size_t>(content_length))
      break;
    if (!wait_fd(fd, POLLIN, timeout_s)) {
      ::close(fd);
      res.status = RpcResult::TIMEOUT;
      res.error = "no answer";
      return res;
    }
    ssize_t n = ::recv(fd, buf, sizeof buf, 0);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      res.error = std::strerror(errno);
      ::close(fd);
      return res;
    }
    if (n == 0) break;   // closed
    resp.append(buf, static_cast<size_t>(n));
    if (header_end == std::string::npos) {
      header_end = resp.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        std::string hdr = resp.substr(0, header_end);
        for (auto& c : hdr) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        const size_t cl = hdr.find("content-length:");
        if (cl != std::string::npos) content_length = std::strtol(hdr.c_str() + cl + 15, nullptr, 10);
      }
    }
  }
  ::close(fd);
  if (header_end == std::string::npos) {
    res.error = "no HTTP response";
    return res;
  }
  // "HTTP/1.x 200 OK"
  const size_t sp = resp.find(' ');
  if (sp == std::string::npos || std::atoi(resp.c_str() + sp + 1) != 200) {
    res.error = "HTTP error";
    return res;
  }
  const std::string xml = resp.substr(header_end + 4);
  if (xml.find("<fault>") != std::string::npos) {
    res.error = "XML-RPC fault";
    return res;
  }
  const auto vals = array_values(xml);
  if (vals.empty()) {
    res.error = "malformed XML-RPC response";
    return res;
  }
  res.status = RpcResult::OK;
  res.code = std::atoi(vals[0].c_str());
  if (vals.size() > 2) res.value = vals[2];
  return res;
}

// ---- PlannerLiveness ------------------------------------------------------------------------------

PlannerLiveness::PlannerLiveness(std::string caller, double period, double timeout, double hung_s)
    : caller_(std::move(caller)), period_(period), timeout_(timeout), hung_s_(hung_s) {}

PlannerLiveness::~PlannerLiveness() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
}

void PlannerLiveness::start() { thread_ = std::thread(&PlannerLiveness::loop, this); }

std::optional<bool> PlannerLiveness::alive(const std::optional<std::string>& node) const {
  if (!node) return std::nullopt;
  std::lock_guard<std::mutex> g(lock_);
  auto it = verdict_.find(*node);
  if (it == verdict_.end()) return std::nullopt;
  return it->second.first;
}

void PlannerLiveness::set_node(const std::string& mode, const std::string& node) {
  std::lock_guard<std::mutex> g(lock_);
  node_[mode] = node;
}

std::optional<std::string> PlannerLiveness::node(const std::string& mode) const {
  std::lock_guard<std::mutex> g(lock_);
  auto it = node_.find(mode);
  if (it == node_.end()) return std::nullopt;
  return it->second;
}

void PlannerLiveness::want(const std::optional<std::string>& node) {
  std::lock_guard<std::mutex> g(lock_);
  want_ = node;
}

void PlannerLiveness::loop() {
  while (ros::ok() && !stop_) {
    std::optional<std::string> node;
    {
      std::lock_guard<std::mutex> g(lock_);
      node = want_;
    }
    if (node) ping(*node);
    ros::Duration(period_).sleep();
  }
}

void PlannerLiveness::set(const std::string& node, std::optional<bool> alive, bool answered) {
  const double now = ros::Time::now().toSec();
  std::lock_guard<std::mutex> g(lock_);
  std::optional<double> last;
  auto it = verdict_.find(node);
  if (it != verdict_.end()) last = it->second.second;
  if (answered) last = now;
  verdict_[node] = {alive, last};
}

RpcResult PlannerLiveness::lookup(const std::string& node) {
  // rosgraph.Master.lookupNode: no timeout of its own in Python (the master is local); 5 s here
  return xmlrpc_call(ros::master::getURI(), "lookupNode", {caller_, node}, 5.0);
}

void PlannerLiveness::ping(const std::string& node) {
  std::string uri;
  auto u = uri_.find(node);
  if (u == uri_.end()) {
    RpcResult r = lookup(node);
    if (r.status != RpcResult::OK) return;   // the master unreachable: no verdict
    if (r.code == -1) {
      set(node, false);                      // not registered: shut down
      return;
    }
    if (r.code != 1) return;                 // (a master failure: no verdict)
    uri = r.value;
    uri_[node] = uri;
  } else {
    uri = u->second;
  }
  RpcResult r = xmlrpc_call(uri, "getPid", {caller_}, timeout_);
  if (r.status == RpcResult::OK) {
    set(node, r.code == 1, r.code == 1);
    return;
  }
  if (r.status == RpcResult::TIMEOUT) {
    std::optional<double> last;
    {
      std::lock_guard<std::mutex> g(lock_);
      auto it = verdict_.find(node);
      if (it != verdict_.end()) last = it->second.second;
    }
    if (last && ros::Time::now().toSec() - *last > hung_s_) set(node, false);   // hung: no answer for hung_s
    return;
  }
  // gone (refused); a restart under the same name re-registers
  uri_.erase(node);
  RpcResult l = lookup(node);
  if (l.status == RpcResult::OK && l.code == 1) {
    uri_[node] = l.value;
    if (l.value != uri) return;   // restarted with a new URI: asked next period
  }
  set(node, false);
}

}  // namespace l4_bridge
