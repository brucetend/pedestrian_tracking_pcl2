import numpy as np

class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):
#F为系统矩阵A, x状态变量；B..u是控制变量
#z
        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]  #shape1是列数
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = 10000*np.eye(self.n) if R is None else R
        self.P = 10000*np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
        #self.x = [-40,50,0,0] if x0 is None else x0
    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))   #刚才这个的维度不一致
        #kalman 增益
        self.x = self.x + np.dot(K, y)
        #状态变量得到
        I = np.eye(self.n)
       # self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
       # 	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
        
        self.P = np.dot(I - np.dot(K, self.H), self.P)

def example():
    dt = 1.0/3

    F = np.array([[1, 0, dt, 0], [0, 1,0, dt], [0, 0, 1,0],[0,0,0,1]])
    H = np.array([1, 0, 0, 0]).reshape(1, 4) #先测量一个
    Q = np.array([[0.05, 0.05, 0.0, 0.0], [0.05, 0.05, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]])
	#R = np.array([[1, 2], [0.5, 0.5]]).reshape(2, 2)
    R = np.array([[1]]).reshape(1, 1)
    x = np.linspace(-10, 1, 1000)
    measurements = - (x**2 + 2*x - 2)  + np.random.normal(0, 2, 1000)

    kf = KalmanFilter(F = F, H = H, Q = Q, R = R)
    predictions = []
    velocity = []

    for z in measurements:
        predictions.append(np.dot(H,  kf.predict())[0])
        kf.update(z)
         #velocity.append(kf.predict())
         #print(velocity)

    import matplotlib.pyplot as plt
    plt.plot(range(len(measurements)), measurements, label = 'Measurements')
    plt.plot(range(len(predictions)), np.array(predictions), label = 'Kalman Filter Prediction')
    plt.legend()
    plt.show()
    

if __name__ == '__main__':
    example()
