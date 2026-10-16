class Filter2:
    def __init__(self, wn=3.4, dt=0.001):
        self.wn = wn
        self.dt = dt
        self.x1 = 0.0   # posición filtrada (ud)
        self.x2 = 0.0   # velocidad filtrada (udp)
    
    def step(self, ur):
        """
        Un paso del filtro (equivalente a los dos integradores + realimentaciones)
        """
        # Ecuaciones del diagrama:
        # error1 = ur - x1
        # error2 = 3.4*error1 - 2*x2
        # x2_dot = 3.4 * error2
        # x1_dot = x2
        
        error1 = ur - self.x1
        error2 = self.wn * error1 - 2.0 * self.x2
        
        x2_dot = self.wn * error2
        x1_dot = self.x2
        
        # Integración Euler (o puedes usar RK4 si quieres más precisión)
        self.x2 += x2_dot * self.dt
        self.x1 += x1_dot * self.dt
        
        return self.x1, self.x2   # ud, udp