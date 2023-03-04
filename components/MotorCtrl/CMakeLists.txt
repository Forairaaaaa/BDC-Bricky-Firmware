# CMakeLists.txt of MotorCtrl component

# Source files directories
set(MotorCtrl_SRC_DIRS
    "."
    "Motor"
)

# Head files directories
set(MotorCtrl_INC_DIRS
    "."
    "Motor"
)

# Public component requirement
set(MotorCtrl_REQUIRES
    "driver"
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
