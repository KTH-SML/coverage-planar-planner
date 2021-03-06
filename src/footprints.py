import numpy as np
import matplotlib.pyplot as plt
import warnings as wrn


class Footprint(object):
    """The Footprint class.
    Abstract class.
    This class implements an abstract footprint object
    for the coverage planner.
    """


    def __str__(self):
        string = self.__class__.__name__
        return string


    def __call__(self, p, n, q, m):
        return self.val(p, n, q, m)


    def val(self, p, n, q, m):
        """Function that returns the value of the footprint.
        
        Args:
            p: Position of the sensor.
            n: Orientation of the sensor.
            q: Position of the landmark.
            m: Orientation of the landmark.
            
        Returns:
            float: The value of the footprint (>=0).
        
        """
        raise NotImplementedError()

        
    def pos_grad(self, p, n, q, m):
        """Function that returns the gradient of the footprint
        with respect to the position.
        
        Args:
            p: Position of the sensor.
            n: Orientation of the sensor.
            q: Position of the landmark.
            m: Orientation of the landmark.
            
        Returns:
            numpy array: The gradient of the footprint
                with respect to the position.
        
        """
        raise NotImplementedError()

        
    def ori_grad(self, p, n, q, m):
        """Function that returns the gradient of the footprint
        with respect to the orientation.
        
        Args:
            p: Position of the sensor.
            n: Orientation of the sensor.
            q: Position of the landmark.
            m: Orientation of the landmark.
            
        Returns:
            numpy array: The gradient of the footprint
                with respect to the orientation.
        
        """
        raise NotImplementedError()

        
    def contour_plot(
            self,
            p=np.zeros(2),
            n=np.array([1.0,0.0]),
            m_func = None,
            xlim = (-1.0,1.0),
            ylim = (-1.0,1.0),
            num_points = 100,
            filename = None
            ):
        """Draw a contour plot of the footprint
        as a function of qx and qy.
        
        Args:
            p: Position of the sensor.
            n: Orientation of the sensor.
            qz_func: Function that returns qz
                as a function of qx and qy (default returns 0.0)
            m_func: Function that returns m
                as a function of q (default returns n)
            xlim: plot limits around px
            ylim: plot limits around py
            num_points: number of points generated for the plot (default 100)
            filename: name of the file where the plot is saved
                (default '[ClassName].pdf')
        
        """
        if m_func == None:
            def m_func(q):
                return n
        if filename == None:
            filename = self.__class__.__name__ + '.pdf'
        xvec = np.linspace(p[0]+xlim[0], p[0]+xlim[1], num=num_points)
        yvec = np.linspace(p[1]+ylim[0], p[1]+ylim[1], num=num_points)
        zmat = np.zeros((len(xvec), len(yvec)))
        for i, x in enumerate(xvec):
	        for j, y in enumerate(yvec):
		        q = np.array([x, y])
		        m = m_func(q)
		        zmat[j][i] = self.val(p, n, q, m)
        cs = plt.contour(xvec,yvec,zmat,20)
        plt.xlabel('$q_x$')
        plt.ylabel('$q_y$')
        plt.axis('equal')
        plt.colorbar(cs)
        plt.grid()
        plt.savefig(filename)
        









class OmnidirectionalFootprint(Footprint):

    def __init__(self, gain=1.0):
        self._GAIN = gain
        
    def val(self, p, n, q, m):
        return 0.5*self._GAIN*np.linalg.norm(p-q)**2
        
    def pos_grad(self, p, n, q, m):
        return self._GAIN*(p-q)
        
    def ori_grad(self, p, n, q, m):
        return np.zeros(2)











class SphericalFootprint(Footprint):
    r"""A footprint with spherical level sets.
    Formula in pseudo-latex is
    
    $f(p,n,q,m)=\norm{p+\beta n-q}^2+\norm{p+\beta m-q}^2$,
    
    where $\beta>0$ is the best distance."""

    def __init__(self, best_distance=1.0, gain=1.0):
        self._GAIN = gain
        self._BEST_DISTANCE = best_distance
        
    def val(self, p, n, q, m):
        BD = self._BEST_DISTANCE
        GAIN = self._GAIN
        return GAIN*(np.linalg.norm(p+BD*n-q)**2 + np.linalg.norm(p+BD*m-q)**2)
        
    def pos_grad(self, p, n, q, m):
        BD = self._BEST_DISTANCE
        GAIN = self._GAIN
        return GAIN*(4*p+2*BD*n+2*BD*m-4*q)
        
    def ori_grad(self, p, n, q, m):
        BD = self._BEST_DISTANCE
        GAIN = self._GAIN
        return GAIN*(2*(p+BD*n-q)*BD)
        
    def contour_plot(self):
        BD = self._BEST_DISTANCE
        xlim = (-0.5*BD,1.5*BD)
        ylim = (-BD,BD)
        Footprint.contour_plot(self, xlim=xlim, ylim=ylim)
        
        
        
        


class EggFootprint(Footprint):
    r"""A footprint with egg shaped level sets.
    Formula in pseudo-latex is
    
    $f(p,n,q,m) = 
        \frac{k_f+k_r}{2} \norm{p+\beta n-q}^2
        + \frac{k_r-k_f}{2}\norm{p+\beta n-q} \paren{
            \norm{p+\beta n-q} + (p+\beta n-q)\T n 
            }
        + \sigma\paren{
            \frac{k_f+k_r}{2} \norm{p+\beta m-q}^2
            + \frac{k_r-k_f}{2}\norm{p+\beta n-q} \paren{
                \norm{p+\beta n-q} + (p+\beta n-q)\T n
                }
            }
    $,
    
    where:
    
        $\beta > 0$ is the best distance;
        $k_f, k_r > 0$ are the front and rear gain respectively;
            usually $k_f < k_r$;
        $\sigma > 0$ is the scale factor of the cost of bad alignment with
            the landmark's orientation.
    
    """

    def __init__(self,
            best_distance=1.0,
            front_gain=0.1,
            rear_gain=1.0,
            orientation_scale_factor=0.0):
        if rear_gain < front_gain:
            wrn.warn('The rear gain is smaller than the front gain.')
        self._BEST_DISTANCE = best_distance
        self._FRONT_GAIN = front_gain
        self._REAR_GAIN = rear_gain
        self._ORIENTATION_SCALE_FACTOR = orientation_scale_factor
       
        
    def val(self, p, n, q, m):
        BD = self._BEST_DISTANCE
        FG = self._FRONT_GAIN
        RG = self._REAR_GAIN
        MG = self._ORIENTATION_SCALE_FACTOR
        vec = p+BD*n-q
        norm = np.linalg.norm(vec)
        one = (FG+RG)/2*norm**2
        two = (RG-FG)/2*norm*vec.dot(n)
        vec = p+BD*m-q
        norm = np.linalg.norm(vec)
        three = (FG+RG)/2*norm**2
        four = (RG-FG)/2*norm*vec.dot(m)
        return one + two + MG*(three + four)
       
        
    def pos_grad(self, p, n, q, m):
        BD = self._BEST_DISTANCE
        FG = self._FRONT_GAIN
        RG = self._REAR_GAIN
        MG = self._ORIENTATION_SCALE_FACTOR
        vec = p+BD*n-q
        norm = np.linalg.norm(vec)
        one = (FG+RG)*vec
        if norm==0:
            two = np.zeros(2)
        else:
            two = (RG-FG)/2*(vec.dot(n)*vec/norm+norm*n)
        vec = p+BD*m-q
        norm = np.linalg.norm(vec)
        three = (FG+RG)*vec
        if norm==0:
            four = np.zeros(2)
        else:
            four = (RG-FG)/2*(vec.dot(m)*vec/norm+norm*m)
        return one + two + MG*(three + four)
       
        
    def ori_grad(self, p, n, q, m):
        BD = self._BEST_DISTANCE
        FG = self._FRONT_GAIN
        RG = self._REAR_GAIN
        vec = p+BD*n-q
        norm = np.linalg.norm(vec)
        one = (FG+RG)*vec
        if norm==0:
            two = np.zeros(2)
        else:
            two = (RG-FG)/2*(vec.dot(n)*vec/norm+norm*n+norm*vec)
        return BD*(one + two)
        
        
    def contour_plot(self, **kwargs):
        BD = self._BEST_DISTANCE
        xlim = (-BD+BD,4.0*BD+BD)
        ylim = (-2.5*BD,2.5*BD)
        Footprint.contour_plot(self, xlim=xlim, ylim=ylim, **kwargs)
        
        



if __name__ == '__main__':
    fp = EggFootprint()
    print fp
    plt.figure()
    #def m_func(q):
    #    return np.array([0,1,0])
    #fp.contour_plot(m_func=m_func)
    fp.contour_plot()
    plt.show()
