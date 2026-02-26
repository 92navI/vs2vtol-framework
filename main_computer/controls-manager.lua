pid = require("advanced_math/pid")

-- Yaw rate PID
pitch_pid = pid.new(0, cfg.Kp.p, cfg.Ki.p, cfg.Kd.p, true)

-- Yaw rate PID
roll_pid = pid.new(0, cfg.Kp.r, cfg.Ki.r, cfg.Kd.r, true)

-- Yaw rate PID
yaw_pid = pid.new(0, cfg.Kp.y, cfg.Ki.y, cfg.Kd.y, true)

-- Main: parse user input into torque commands
function parseUserInput(input, dt)
    dt = dt or 0.05

    -- Stick → desired tilt angles
    local roll_angle = input.roll * cfg.MAX_ROLL
    local pitch_angle = input.pitch * cfg.MAX_PITCH
    local yaw_rate_sp = input.yaw * cfg.YAW_RATE

    -- Ship data
    local q_ship = ship.getQuaternion()

    -- --- VECTOR-BASED ATTITUDE ERROR ---
    local world_up = vector.new(0, 1, 0)

    -- Desired up vector in world frame
    local q_target = quaternion.fromEuler(roll_angle, 0, pitch_angle)
    local target_up = q_target * world_up
    -- print("target_up: "..tostring(target_up))

    -- Current up vector
    local current_up = q_ship * world_up
    -- print("current_up: "..tostring(current_up))

    -- Cross product gives rotation axis *and* magnitude
    local att_error = current_up:cross(target_up)

    -- print("\natt_error: ")
    -- print(att_error.x)
    -- print(att_error.z)

    local pitch = pitch_pid:step(att_error.x, dt)

    local roll = roll_pid:step(att_error.z, dt)

    -- Combine torques
    local alpha_cmd = {
        x = deadzone(pitch, 0.00001),
        y = 0,
        z = deadzone(roll, 0.00001)
    }

    -- Normalize / clamp
    local ctrl = {
        x = clamp(alpha_cmd.x / cfg.MAX_ROLL * 2, -1, 1),
        y = clamp(alpha_cmd.y / cfg.YAW_RATE * 2, -1, 1),
        z = clamp(alpha_cmd.z / cfg.MAX_PITCH * 2, -1, 1),
        t = input.thrust
    }

    return ctrl
end

-- Helper: clamp
function clamp(x, min_val, max_val)
    return math.max(min_val, math.min(max_val, deadzone(x, 0.00001)))
end

-- Helper: deadzone
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
    
    if i < 0 then s = -1
    else s = 1
    end

    return (x - dz) / (1 - dz) * s
end


function duplicate(v)
    return vector.new(v.x, v.y, v.z)
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

return {
    parseUserInput = parseUserInput,
    parseControllerValues = parseControllerValues
}
