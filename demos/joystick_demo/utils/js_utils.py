import pygame

class JoyStick_helper:
    def __init__(self, js_id=0):
        pygame.display.init()
        pygame.joystick.init()
        self.js = pygame.joystick.Joystick(js_id)
        
        self.dead = 0.2

    def get_state(self):
        pygame.event.pump()
        llr = self.js.get_axis(0)
        lud = self.js.get_axis(1)
        rlr = self.js.get_axis(2)
        rud = self.js.get_axis(3)
        
        llr = llr*(not (abs(llr) < self.dead))
        lud = lud*(not (abs(lud) < self.dead))
        rlr = rlr*(not (abs(rlr) < self.dead))
        rud = rud*(not (abs(rud) < self.dead))
        
        lshoulder = self.js.get_button(4)
        rshoulder = self.js.get_button(5)

        return [llr, lud, rlr, rud], lshoulder, rshoulder
