import numpy as np

class OptimizeFlyby():
    def __init__(self):
        pass

    def optimizePath(self,path: list = [], planner = None, config: dict = {}):
        """Returns name of node and the Überschleifwert (r) belonging to it.
        
        Args:
            path (list): Path to optimize.
            planner (Planner): Planner to use.
            config (dict): Configuration.
        """


        if len(path) <= 2:
            return path, [0 for _ in path]

        for i in range(1, len(path) - 1):
            P2 = np.array(path[i])
            P1 = np.array(path[i - 1])
            P3 = np.array(path[i + 1])
            
            S = config['r'] * (P2 - P1) + P2
            E = config['r'] * (P3 - P2) + P2
            
        
        return path, [0 for _ in path]
