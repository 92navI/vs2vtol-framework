-- Parse json config
local cfg_raw = "" 
for l in io.open("config.json", "r"):lines() do
    cfg_raw = cfg_raw..l
end
cfg = textutils.unserialiseJSON(cfg_raw)

local controlsManager = require('controls-manager')

-- Wrap needed peripherals
ctc = peripheral.wrap(cfg.controller_id)
local modem = peripheral.wrap(cfg.modem_id)
local monitor = peripheral.wrap(cfg.monitor_id)

-- Set up monitor for logging
monitor.setTextScale(0.5)
monitor.setCursorPos(1, 1)
monitor.setCursorBlink(true)
monitor.clear()
term.redirect(monitor)

while true do
    while true do
        if ctc.hasUser() then

            -- Parse inputs
            local input = controlsManager.parseControllerValues()
            ctrl = controlsManager.parseUserInput(input)
        else

            -- Level aircraft
            ctrl = controlsManager.parseUserInput({
                yaw = 0,
                thrust = 0,
                roll = 0,
                pitch = 0
            })
        end

        print(ctrl.p)
        print(ctrl.r)
        print(ctrl.y)
        print(ctrl.t)
        print()

        modem.transmit(1488, 0, controlsManager.compileControlPayload(ctrl))

        os.sleep(0)
    end
end
