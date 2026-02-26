local CONFIG = {}

CONFIG.MODEM_PATH = "top"
CONFIG.CONTROLLER_PATH = "tweaked_controller_3"
CONFIG.MONITOR_PATH = "monitor_2"

-- Ship settings
CONFIG.MAX_ROLL  = math.rad(30) -- deg
CONFIG.MAX_PITCH = math.rad(20) -- deg
CONFIG.YAW_RATE  = math.rad(90) -- deg/s

CONFIG.Kp = {
    p = CONFIG.MAX_PITCH*2,
    r = CONFIG.MAX_ROLL*2,
    y = 0.8,
}

CONFIG.Ki = {
    p = 0,
    r = 0,
    y = 0,
}

CONFIG.Kd = {
    p = 0.3,
    r = 0.3,
    y = 0,
}

CONFIG.ROTOR_CONFIG = {}

CONFIG.ROTOR_CONFIG.MAX_FLAP_ANGLE = 22.5
CONFIG.ROTOR_CONFIG.THRUST_ANGLE = 5

return CONFIG
