import numpy as np

P0_2 = np.array([[3, 0], [5, 0], [6, 1], [7, 3]])
print(P0_2)
print(P0_2.shape)
q0 = np.array(np.ones((P0_2.shape[0],1)))
Q = np.concatenate([P0_2,q0], axis=1)
print(Q)
q, r = np.linalg.qr(Q)
print(r)
print(q)
print("SUM: {0}".format(np.sum(Q*Q, axis=1)))
b = np.dot(q.T, np.sum(Q*Q, axis=1))
z = np.dot(np.linalg.inv(r), b)
print(z)
y = 0.5*z[0:2]
print(z)
print(np.sqrt(z[2]+y.dot(y)))