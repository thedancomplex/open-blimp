import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import pygame

class JoyStick_helper:
    def __init__(self, js_id=0):

        pygame.display.init()
        pygame.joystick.init()
        self.js = pygame.joystick.Joystick(js_id)
        
        self.dead = 0.4

    def get_state(self):
        pygame.event.pump()
        llr = self.js.get_axis(0)
        lud = self.js.get_axis(1)
        rlr = self.js.get_axis(2)
        rud = self.js.get_axis(3)
        
        llr *= not (abs(llr) < self.dead)
        lud *= not (abs(lud) < self.dead)
        rlr *= not (abs(rlr) < self.dead)
        rud *= not (abs(rud) < self.dead)

        llr += self.dead*(llr < 0) - self.dead*(llr > 0)
        lud += self.dead*(lud < 0) - self.dead*(lud > 0)
        rlr += self.dead*(rlr < 0) - self.dead*(rlr > 0)
        rud += self.dead*(rud < 0) - self.dead*(rud > 0)

        lshoulder = self.js.get_button(4)
        rshoulder = self.js.get_button(5)

        return [llr, lud, rlr, rud], lshoulder, rshoulder
