#include "utils.h"

namespace fs = boost::filesystem;

std::string get_config_directory() {
    // First, check the environment variable HAPTIC_WRIST_CONFIG_DIR
    const char* config_dir_env = std::getenv("HAPTIC_WRIST_CONFIG_DIR");
    if (config_dir_env) {
        fs::path config_path(config_dir_env);
        if (fs::exists(config_path) && fs::is_directory(config_path)) {
            return config_path.string();
        } else {
            std::cerr << "Environment variable HAPTIC_WRIST_CONFIG_DIR is set, but the directory doesn't exist or is "
                         "invalid.\n";
        }
    }

    // Second, check for the default user config directory: ~/.config/haptic_wrist
    fs::path user_config_dir = fs::path(getenv("HOME")) / ".config" / "haptic_wrist";
    if (fs::exists(user_config_dir) && fs::is_directory(user_config_dir)) {
        return user_config_dir.string();
    }

    // Finally, fallback to the system-wide config directory: /etc/haptic_wrist
    fs::path system_config_dir = "/etc/haptic_wrist";
    if (fs::exists(system_config_dir) && fs::is_directory(system_config_dir)) {
        return system_config_dir.string();
    }

    // If no valid config directory was found, print an error
    std::cerr << "No valid configuration directory found.\n";
    return "";
}
