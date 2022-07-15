class Signal : 

    strength_decay = 0.9

    def __init__(self, strength, direction) : 
        self.strength = strength
        self.direction = direction
        self.signal = 0
    def travel(self, step) : 
        self.strength *= Signal.strength_decay
        self.signal += Signal.noise(step, self.strength)
    

    @staticmethod
    def noise(step, strength) : 
        '''
        Adds noise to the step
        less strength = more noise
        '''

        # Add noise here
        return step
    