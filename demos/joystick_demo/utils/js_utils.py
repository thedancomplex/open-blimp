import pygame

class JoyStick_helper:
    def __init__(self, js_id=0):
        pygame.display.init()
        pygame.joystick.init()
        self.js = pygame.joystick.Joystick(js_id)
        
        self.button_state = False
        self.toggle_state = False
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
        
        # get debounced button value
        button_event = False
        if self.js.get_button(3) and not self.button_state: 
            self.button_state = True
            self.toggle_state = not self.toggle_state
            button_event = True
          
        elif not self.js.get_button(3) and self.button_state:
            self.button_state = False

        return [llr, lud, rlr, rud], button_event, self.toggle_state
