// comms: mission state store (C++ port of comms/mission_state_utils.py)
//
// The Python original persisted a small, flat JSON object next to the source
// file. This is a dependency-free reimplementation: the schema is a flat map of
// key -> JSON scalar (string / number / bool / null), stored as raw JSON tokens
// so values round-trip without a full JSON parser.

#ifndef COMMS__MISSION_STATE_HPP_
#define COMMS__MISSION_STATE_HPP_

#include <map>
#include <string>

namespace comms
{

// Absolute path to the state file (alongside this package's sources at build
// time; overridable at runtime via the COMMS_STATE_FILE environment variable).
std::string state_file_path();

// A flat state object: key -> raw JSON token (e.g. "\"shutdown\"", "31", "null").
using State = std::map<std::string, std::string>;

// Load the state file, merged over the defaults. Never throws; returns defaults
// on any error or if the file does not exist.
State load_state();

// Set one key to a value and persist the whole object atomically-ish.
void update_state_raw(const std::string & key, const std::string & raw_json_value);
void update_state(const std::string & key, const std::string & string_value);
void update_state(const std::string & key, long long int_value);
void update_state(const std::string & key, double double_value);
void update_state(const std::string & key, bool bool_value);
void update_state_null(const std::string & key);

// Read a key as a string (strips surrounding quotes). Returns "" if absent/null.
std::string get_string(const State & state, const std::string & key);

}  // namespace comms

#endif  // COMMS__MISSION_STATE_HPP_
