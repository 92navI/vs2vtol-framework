cfg = require('config')
controlsManager = require('controls-manager')

modem = peripheral.wrap('top')
ctc = peripheral.wrap(cfg.CONTROLLER_PATH)
monitor = peripheral.wrap(cfg.MONITOR_PATH)

monitor.setTextScale(0.5)
monitor.setCursorPos(1, 1)
monitor.setCursorBlink(true)
monitor.clear()
term.redirect(monitor)

print("started")

while true do
    while true do
        if ctc.hasUser() then
            local input = controlsManager.parseControllerValues()
            ctrl = controlsManager.parseUserInput(input)

            print(ctrl.x)
            print(ctrl.y)
            print(ctrl.z)
            print()

        else
            ctrl = controlsManager.parseUserInput({
                yaw = 0,
                thrust = 0,
                roll = 0,
                pitch = 0
            })
            
            print(ctrl.x)
            print(ctrl.y)
            print(ctrl.z)
            print()

        end

        modem.transmit(1488, 0, {
            q_ship = ship.getQuaternion(),
            config = cfg.ROTOR_CONFIG,
            ctrl = ctrl
        })

        os.sleep(0)
    end
end
