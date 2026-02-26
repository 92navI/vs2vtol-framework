local pid = require("advanced_math/pid")

-- Pitch rate PID
local pitch_pid = pid.new(0, cfg.Kp.p, cfg.Ki.p, cfg.Kd.p, true)

-- Roll rate PID
local roll_pid = pid.new(0, cfg.Kp.r, cfg.Ki.r, cfg.Kd.r, true)

-- Yaw rate PID
local yaw_pid = pid.new(0, cfg.Kp.y, cfg.Ki.y, cfg.Kd.y, true)

-- Parse user input into torque commands
function parseUserInput(input, dt)
    local dt = dt or 0.05

    -- Calculate desired tilt values
    local roll_angle = input.roll * math.rad(cfg.max_roll)
    local pitch_angle = input.pitch * math.rad(cfg.max_pitch)
    local yaw_rate_sp = input.yaw * math.rad(cfg.yaw_rate)

    -- Get ship rotation
    local q_ship = ship.getQuaternion()

    -- --- VECTOR-BASED ATTITUDE ERROR ---
    local world_up = vector.new(0, 1, 0)

    -- Desired up vector in world frame
    local q_target = quaternion.fromEuler(roll_angle, 0, pitch_angle)
    local target_up = q_target * world_up

    -- Current up vector
    local current_up = q_ship * world_up

    -- Cross product gives rotation vector
    local att_error = current_up:cross(target_up)

    -- Update pid controllers
    local p = pitch_pid:step(att_error.z, dt)
    local r = roll_pid:step(att_error.x, dt)

    -- Combine torques
    local alpha_cmd = {
        x = deadzone(p, 0.00001),
        y = 0,
        z = deadzone(r, 0.00001)
    }

    -- Normalize / clamp
    local ctrl = {
        p = clamp(alpha_cmd.x / cfg.Kp.p*2, -1, 1),
        r = clamp(alpha_cmd.z / cfg.Kp.r*2, -1, 1),
        y = clamp(alpha_cmd.y / cfg.Kp.y*2, -1, 1),
        t = input.thrust
    }

    return ctrl
end

-- Controller stick parsing
function parseControllerValues()
    return {
        yaw = smoothDeadzone(ctc.getAxis(1), 0.25),
        thrust = smoothDeadzone(ctc.getAxis(2), 0.25),
        roll = smoothDeadzone(ctc.getAxis(3), 0.25),
        pitch = smoothDeadzone(ctc.getAxis(4), 0.25)
    }
end

function compileControlPayload(ctrl)

    local f_ctrl = {}

    for k, v in pairs(cfg.rotor_modifiers) do
        f_ctrl[k] = {
            l = {
                p = ctrl.p * v.l.p,
                r = ctrl.r * v.l.r,
                t = ctrl.t * v.l.t
            },
            r = {
                p = ctrl.p * v.r.p,
                r = ctrl.r * v.r.r,
                t = ctrl.t * v.r.t
            }
        }
    end

    return {
        q_ship = ship.getQuaternion(),
        cfg = cfg,
        ctrl = f_ctrl
    }
end

-- Helper functions
function clamp(x, min_val, max_val)
    return math.max(min_val, math.min(max_val, deadzone(x, 0.00001)))
end

function deadzone(val, threshold)
    if math.abs(val) > threshold then
        return val
    else
        return 0
    end
end

function smoothDeadzone(i, dz)
    local x = math.abs(i)
    if x <= dz then
        return 0
    end

    if i < 0 then
        s = -1
    else
        s = 1
    end

    return (x - dz) / (1 - dz) * s
end

function duplicate(v)
    return vector.new(v.x, v.y, v.z)
end

return {
    parseUserInput = parseUserInput,
    parseControllerValues = parseControllerValues,
    compileControlPayload = compileControlPayload
}
