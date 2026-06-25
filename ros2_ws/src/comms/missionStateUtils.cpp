#include <iostream>
#include <filesystem>
#include <nlohmann/json.hpp> //json library


// __FILE__ now is the current .cpp file path
const std::filesystem::path STATE_FILE = std::filesystem::path(__FILE__).parent_path() / "missionState.json";

const nlohmann::json DEFAULTS = {
    {"last_command", nullptr},
    {"timestamp", nullptr},
    {"last_sender_sysid", nullptr},
    {"last_sender_compid", nullptr},
    {"pending_actions", nullptr},
};

nlohmann::json load_state()
{
    try 
    {
        if (std::filesystem::exists (STATE_FILE)) 
        {
            std::ifstream file(STATE_FILE);

            nlohmann::json loaded;
            file >> loaded;

            return merge_dicts(DEFAULS, loaded);
        }
    }

    catch
    {
        continue;
    }

    return result
}

nlohmann::json merge_dicts(const nlohmann:json& defaults, const nlohmann:json& override_data) // we are returning nlohmann::json
{
    nlohmann::json result = defaults;

    for (auto& [key, value] : override_data.items()) //loops through every key and value pair in override_data map
    {
        if (value.is_object() && 
        result.contains(keys) && 
        result[key].is_object()) result[key] = merge_dicts(result[key], value);

        else result[key] = value;
    }
    return result;
}