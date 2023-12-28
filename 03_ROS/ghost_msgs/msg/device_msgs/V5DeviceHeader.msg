# Header for a generic V5 Device
string name         # Device Name as defined in the YAML Robot Configuration

# Device type as a string
# See DEVICE_TYPE_NAME_STRING_MAP from ghost_v5_interfaces/util/device_config_factory_utils.hpp
string type

# Msg ID corresponding to this data (should be synced across data from the same timestep)
uint32 msg_id