local ship = ship
local quaternion = quaternion

local controlsChannel = 1488 -- the channel at which the hull computer is sending signals

local thisRotor = os.getComputerLabel() -- for parity between rotors, should match "left_rotor" or "right_rotor"
if thisRotor == nil then
    error("Rotor label cannot be null!")
end

local MAX_FLAP_ANGLE = 22.5 -- constant

local modem = peripheral.wrap("top")
modem.open(controlsChannel)

local flaps = {
    left = peripheral.wrap("left"),
    right = peripheral.wrap("right")
}

local function reconstructQuat(q)
    return quaternion.fromComponents(q.v.x, q.v.y, q.v.z, q.a)
end

local function clamp(x, min_val, max_val)
    return math.max(min_val, math.min(max_val, deadzone(x, 0.001)))
end

function deadzone(val, threshold)
    if math.abs(val) > threshold then
        return val
    else
        return 0
    end
end

local function bladeAngleCoeff(q_ship)
    local q_rel = q_ship:inverse() * ship.getQuaternion()

    -- Normalize hemisphere
    if q_rel.a < 0 then
        q_rel.a = -q_rel.a
        q_rel.v.x = -q_rel.v.x
        q_rel.v.y = -q_rel.v.y
        q_rel.v.z = -q_rel.v.z
    end

    local angle = 2 * math.acos(math.min(1, q_rel.a)) -- [0, π]
    local s = math.sqrt(1 - q_rel.a * q_rel.a)

    if s < 1e-6 then
        return 0
    end

    local axis = {
        x = q_rel.v.x / s,
        y = q_rel.v.y / s,
        z = q_rel.v.z / s
    }

    -- PROJECT rotation onto blade axis (Y)
    local signedAngle = angle * axis.y

    -- Normalize to coefficient
    return math.cos(signedAngle)
end

-- Bad practice for unevenness but i'll keep it for now

-- local function getAngleNoThrust(angle_coef, ctrl, cfg)
--     local fpitch = angle_coef * ctrl.p * (cfg.MAX_PITCH_ANGLE)
--     local froll = (1 - math.abs(angle_coef)) * ctrl.r * (cfg.MAX_ROLL_ANGLE)

--     return fpitch + froll
-- end

-- local function getMaxAngleNoThrust(ctrl, cfg)
--     return cfg.MAX_FLAP_ANGLE -
--                math.abs(
--             math.max(getAngleNoThrust(1, ctrl.p, 0, cfg), getAngleNoThrust(0, 0, math.abs(ctrl.r), cfg)))
-- end

local function calculateBladeAngle(angle_coef, ctrl, cfg)

    local fpitch = angle_coef * ctrl.p * (cfg.max_pitch)
    local froll = (1 - math.abs(angle_coef)) * ctrl.r * (cfg.max_roll)
    local fthrust = ctrl.t * cfg.max_thrust

    return clamp(fpitch + froll + fthrust, -MAX_FLAP_ANGLE, MAX_FLAP_ANGLE)
end

while true do
    local _, _, channel, _, payload = os.pullEvent("modem_message")
    if channel == controlsChannel then

        -- Get neeeded values from payload
        local cfg = payload.cfg.rotor_config
        local ctrl = payload.ctrl[thisRotor]
        if ctrl == nil then
            error("This rotor name is not valid")
        end

        -- Calculate the angle between rotor and hull
        local coef = bladeAngleCoeff(reconstructQuat(payload.q_ship))

        -- Apply flap angles
        flaps.left.setAngle(calculateBladeAngle(coef, ctrl.l, cfg))
        flaps.right.setAngle(calculateBladeAngle(coef, ctrl.r, cfg))
    end

    sleep(0)
end
