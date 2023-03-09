# CMakeLists.txt of GearMotor component

# Source files directories
set(GearMotor_SRC_DIRS
    "."
    "Motor"
    "Encoder"
)

# Head files directories
set(GearMotor_INC_DIRS
    "."
    "Motor"
    "Encoder"
)

# Public component requirement
set(GearMotor_REQUIRES
    "driver"
    "esp_timer"
    "bdc_motor"
)

# Private component requirement
set(GearMotor_PRIV_REQUIRES
)

# Register component
idf_component_register(SRC_DIRS ${GearMotor_SRC_DIRS}
                  INCLUDE_DIRS ${GearMotor_INC_DIRS}
                  REQUIRES ${GearMotor_REQUIRES}
                  PRIV_REQUIRES ${GearMotor_PRIV_REQUIRES}
)

