#include "comms/mission_state.hpp"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

namespace comms
{
namespace
{
// Default schema (keys present even if never written), matching the Python DEFAULTS.
State defaults()
{
  return {
    {"last_command", "null"},
    {"timestamp", "null"},
    {"last_sender_sysid", "null"},
    {"last_sender_compid", "null"},
    {"pending_action", "null"},
  };
}

std::string trim(const std::string & s)
{
  size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) {
    ++a;
  }
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) {
    --b;
  }
  return s.substr(a, b - a);
}

std::string json_escape(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2);
  out.push_back('"');
  for (char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out.push_back(c);
    }
  }
  out.push_back('"');
  return out;
}

// Parse a flat JSON object into key -> raw token. Best-effort; on malformed
// input returns whatever was parsed so far.
State parse_flat_object(const std::string & text)
{
  State result;
  size_t i = text.find('{');
  if (i == std::string::npos) {
    return result;
  }
  ++i;
  const size_t n = text.size();
  while (i < n) {
    // Skip whitespace and separators.
    while (i < n && (std::isspace(static_cast<unsigned char>(text[i])) || text[i] == ',')) {
      ++i;
    }
    if (i >= n || text[i] == '}') {
      break;
    }
    if (text[i] != '"') {
      break;  // malformed
    }
    // Parse key string.
    std::string key;
    ++i;  // opening quote
    while (i < n && text[i] != '"') {
      if (text[i] == '\\' && i + 1 < n) {
        ++i;
      }
      key.push_back(text[i]);
      ++i;
    }
    ++i;  // closing quote
    // Skip whitespace and ':'.
    while (i < n && (std::isspace(static_cast<unsigned char>(text[i])) || text[i] == ':')) {
      ++i;
    }
    // Parse value token.
    std::string value;
    if (i < n && text[i] == '"') {
      value.push_back('"');
      ++i;
      while (i < n && text[i] != '"') {
        if (text[i] == '\\' && i + 1 < n) {
          value.push_back(text[i]);
          ++i;
        }
        value.push_back(text[i]);
        ++i;
      }
      if (i < n) {
        value.push_back('"');
        ++i;
      }
    } else {
      while (i < n && text[i] != ',' && text[i] != '}') {
        value.push_back(text[i]);
        ++i;
      }
      value = trim(value);
    }
    if (!key.empty()) {
      result[key] = value;
    }
  }
  return result;
}

State merge(const State & base, const State & override_data)
{
  State result = base;
  for (const auto & [k, v] : override_data) {
    result[k] = v;
  }
  return result;
}

void write_state(const State & state)
{
  std::ostringstream oss;
  oss << "{\n";
  size_t idx = 0;
  for (const auto & [k, v] : state) {
    oss << "  " << json_escape(k) << ": " << v;
    if (++idx < state.size()) {
      oss << ",";
    }
    oss << "\n";
  }
  oss << "}\n";
  std::ofstream file(state_file_path());
  file << oss.str();
}
}  // namespace

std::string state_file_path()
{
  if (const char * env = std::getenv("COMMS_STATE_FILE")) {
    return env;
  }
#ifdef COMMS_STATE_FILE_DEFAULT
  return COMMS_STATE_FILE_DEFAULT;
#else
  return "mission_state.json";
#endif
}

State load_state()
{
  std::ifstream file(state_file_path());
  if (!file) {
    return defaults();
  }
  std::stringstream ss;
  ss << file.rdbuf();
  const State loaded = parse_flat_object(ss.str());
  if (loaded.empty()) {
    return defaults();
  }
  return merge(defaults(), loaded);
}

void update_state_raw(const std::string & key, const std::string & raw_json_value)
{
  State state = load_state();
  state[key] = raw_json_value;
  write_state(state);
}

void update_state(const std::string & key, const std::string & string_value)
{
  update_state_raw(key, json_escape(string_value));
}

void update_state(const std::string & key, long long int_value)
{
  update_state_raw(key, std::to_string(int_value));
}

void update_state(const std::string & key, double double_value)
{
  update_state_raw(key, std::to_string(double_value));
}

void update_state(const std::string & key, bool bool_value)
{
  update_state_raw(key, bool_value ? "true" : "false");
}

void update_state_null(const std::string & key)
{
  update_state_raw(key, "null");
}

std::string get_string(const State & state, const std::string & key)
{
  auto it = state.find(key);
  if (it == state.end()) {
    return "";
  }
  const std::string & raw = it->second;
  if (raw == "null" || raw.empty()) {
    return "";
  }
  if (raw.size() >= 2 && raw.front() == '"' && raw.back() == '"') {
    return raw.substr(1, raw.size() - 2);
  }
  return raw;
}

}  // namespace comms
