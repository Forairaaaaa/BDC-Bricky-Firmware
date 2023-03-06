# CMakeLists.txt of MotorCtrl component

# Source files directories
set(MotorCtrl_SRC_DIRS
    "."
    "Motor"
    "Encoder"
)

# Head files directories
set(MotorCtrl_INC_DIRS
    "."
    "Motor"
    "Encoder"
)

# Public component requirement
set(MotorCtrl_REQUIRES
    "driver"
    "esp_timer"
    "bdc_motor"
)

# Private component requirement
set(MotorCtrl_PRIV_REQUIRES
)

# Register component
idf_component_register(SRC_DIRS ${MotorCtrl_SRC_DIRS}
                  INCLUDE_DIRS ${MotorCtrl_INC_DIRS}
                  REQUIRES ${MotorCtrl_REQUIRES}
                  PRIV_REQUIRES ${MotorCtrl_PRIV_REQUIRES}
)
