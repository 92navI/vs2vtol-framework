m = peripheral.wrap('top');
l_f = peripheral.wrap('left')
r_f = peripheral.wrap('right')

m.open(1488)

function qautDot(a, b)
    return a.a * b.a + a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z
end

function reconstructQuat(q) 
    return quaternion.fromComponents(q.v.x, q.v.y, q.v.z, q.a)
end

-- Helper: clamp
function clamp(x, min_val, max_val)
    return math.max(min_val, math.min(max_val, deadzone(x, 0.001)))
end

-- Helper: deadzone
function deadzone(val, threshold)
    if math.abs(val) > threshold then
        return val
    else
        return 0
    end
end

local function bladeAngleCoefficient(q_ship)
    local q_rotor = ship.getQuaternion()
    local q_rel = q_ship:inverse() * q_rotor

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

function getAngleNoThrust(angle_coef, pitch_coef, roll_coef, cfg)
    local fpitch = angle_coef * pitch_coef * (cfg.MAX_FLAP_ANGLE - cfg.THRUST_ANGLE)
    local froll = (1 - math.abs(angle_coef)) * roll_coef * (cfg.MAX_FLAP_ANGLE - cfg.THRUST_ANGLE)
    return fpitch + froll
end

function getMaxAngleNoThrust(pitch_coef, roll_coef, cfg)
    return cfg.MAX_FLAP_ANGLE -
               math.abs(
            math.max(getAngleNoThrust(1, pitch_coef, 0, cfg),
                getAngleNoThrust(0, 0, math.abs(roll_coef), cfg)))
end

function calculateBladeAngle(angle_coef, pitch_coef, roll_coef, thrust, cfg)
    local pitch_and_roll = getAngleNoThrust(angle_coef, pitch_coef, roll_coef, cfg)

    local angle_left = getMaxAngleNoThrust(pitch_coef, roll_coef, cfg)
    local fthrust = thrust * angle_left

    return clamp(pitch_and_roll + fthrust, -cfg.MAX_FLAP_ANGLE, cfg.MAX_FLAP_ANGLE)
end


while true do
    local _, _, channel, _, payload = os.pullEvent("modem_message")
    if channel == 1488 then
        cfg = payload.config
        local coef = bladeAngleCoefficient(reconstructQuat(payload.q_ship))

        l_f.setAngle(calculateBladeAngle(coef, payload.ctrl.z, payload.ctrl.x, payload.ctrl.t, cfg))
        r_f.setAngle(calculateBladeAngle(coef, payload.ctrl.z, -payload.ctrl.x, -payload.ctrl.t, cfg))
    end

    sleep(0)
end
