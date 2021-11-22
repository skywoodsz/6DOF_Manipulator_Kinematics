import numpy as np
from numpy import linalg as la
def pk1(p, q, r, w):
    u = p - r
    v = q - r
    uPrime = u - w * w.transpose() * u
    vPrime = v - w * w.transpose() * v
    para1 = w.transpose()*(np.cross(uPrime.transpose(), vPrime.transpose())).transpose()
    para2 = uPrime.transpose()*vPrime
    return np.arctan2(para1, para2)

def pk2(p, q, r, w2, w1):
    u = p - r
    v = q - r
    alpha = ((np.transpose(w1)*w2) * np.transpose(w2)*u - np.transpose(w1)*v)/((np.transpose(w1)*w2) * (np.transpose(w1)*w2) - 1)
    alpha = alpha[0].item(0)
    print('alpha:')
    print(alpha)
    
    beta = ((np.transpose(w1)*w2)*np.transpose(w1)*v - np.transpose(w2)*u)/((np.transpose(w1)*w2) * (np.transpose(w1)*w2) - 1)
    beta = beta[0].item(0)
    print('beta:')
    print(beta)
    
    a = (la.norm(u)*la.norm(u) - alpha*alpha - beta*beta - 2*alpha*beta*np.transpose(w1)*w2)
    b = (np.power(la.norm(np.cross(np.transpose(w1), np.transpose(w2))), 2))
    gammaSquare = a/b
    if gammaSquare.all() < 0:
        return 'None'
    else:
        gamma = np.sqrt(gammaSquare).item(0)
        negativeGamma = -gamma
        print('gamma:')
        print(gamma)
        
        z1 = alpha * w1 + beta * w2 + gamma*(np.transpose(np.cross(np.transpose(w1), np.transpose(w2))))
        z2 = alpha * w1 + beta * w2 + negativeGamma*(np.transpose(np.cross(np.transpose(w1), np.transpose(w2))))
        c1 = z1 + r
        c2 = z2 + r
        allC = (c1, c2)
        i = 0
        theta = [None] * len(allC)
        print('c1:')
        print(c1)
        
        for c in allC:
            theta[i] = (-pk1(c, q, r, w1)[0].item(0), -pk1(p, c, r, w2)[0].item(0))
            i += 1
        return theta

p = np.matrix([[0],[-1],[1]])
q = np.matrix([[1],[1],[0]])
r = np.matrix([[0],[0],[0]])
w1 = np.matrix([[0],[0],[1]])
w2 = np.matrix([[0],[1],[0]])
thetas = pk2(p,q,r,w2,w1)
print("thetas12:")
print(thetas)
