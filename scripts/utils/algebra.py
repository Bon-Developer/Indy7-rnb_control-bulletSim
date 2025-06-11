from .rotation_utils import *
from .traj_utils import *
from abc import abstractmethod
import scipy

class Algebra:
    def __init__(self, dim):
        self.dim = dim
        self.alg_dim = dim
        
    ## 
    # @brief calculate difference in algebra
    @abstractmethod
    def diff_in_alg(self, X0, Xlist):
        raise(NotImplementedError("diff_in_alg is not implemented for {}".format(type(self))))
        
    ## 
    # @brief add difference in algebra and return to original space
    @abstractmethod
    def add_from_alg(self, X0, dYlist):
        raise(NotImplementedError("diff_in_alg is not implemented for {}".format(type(self))))
    

class Euclidean(Algebra):
    def diff_in_alg(self, X0, Xlist):
        return [X-X0 for X in Xlist]
    
    def add_from_alg(self, X0, dYlist):
        return [X0+dY for dY in dYlist]
    
class SOgroup(Algebra):
    def __init__(self, dim, alg_dim):
        self.dim = dim
        self.alg_dim = alg_dim
    
    @abstractmethod
    def to_Rot(self, X):
        raise(NotImplementedError(""))
        
    @abstractmethod
    def from_Rot(self, R):
        raise(NotImplementedError(""))
        
    def diff_in_alg(self, X0, Xlist):
        R0 = self.to_Rot(X0)
        Ylist = []
        for X in Xlist:
            R1 = self.to_Rot(X)
            R01 = np.matmul(R0.T, R1)
            w_hat = scipy.linalg.logm(R01)
            w = w_hat[2,1], w_hat[0,2], w_hat[1,0]
            Y = np.matmul(R0, w)
            Ylist.append(Y)
        return Ylist
    
    def add_from_alg(self, X0, dYlist):
        R0 = self.to_Rot(X0)
        Xlist = []
        for dY in dYlist:            
            w = np.matmul(R0.T, dY)
            dR = scipy.linalg.expm(np.cross(np.identity(3), w))
            X = self.from_Rot(np.matmul(R0, dR))
            Xlist.append(X)
        return Xlist
    
class RotationUVW(SOgroup):
    def __init__(self):
        SOgroup.__init__(self, 3, 3)
    
    @abstractmethod
    def to_Rot(self, X):
        return Rot_zyx(*list(reversed(X)))
        
    @abstractmethod
    def from_Rot(self, R):
        return list(reversed(Rot2zyx(R)))
        
class Combined(Algebra):
    ##
    # @param alg_list list of Algebra instance
    def __init__(self, alg_list):
        self.alg_list = alg_list
        self.dim_list = [alg.dim for alg in alg_list]
        self.alg_dim_list = [alg.alg_dim for alg in alg_list]
        self.dim = np.sum(self.dim_list)
        self.alg_dim = np.sum(self.alg_dim_list)
        
    def diff_in_alg(self, X0, Xlist):
        Ylist = []
        dim_accum = 0
        for alg, dim in zip(self.alg_list, self.dim_list):
            Ylist.append(alg.diff_in_alg(X0[dim_accum:dim_accum+dim], 
                                         [X[dim_accum:dim_accum+dim] for X in Xlist])
                        )
            dim_accum+=dim
        return np.concatenate(Ylist, axis=-1)
    
    def add_from_alg(self, X0, dYlist):
        Xlist = []
        dim_accum = 0
        dim_out_accum = 0
        for alg, dim, dim_out in zip(self.alg_list, self.dim_list, self.alg_dim_list):
            Xlist.append(alg.add_from_alg(X0[dim_accum:dim_accum+dim], 
                                          [dY[dim_out_accum:dim_out_accum+dim_out] for dY in dYlist]))
            dim_accum+=dim
            dim_out_accum+=dim_out
        return np.concatenate(Xlist, axis=-1)