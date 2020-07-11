import numpy as np
from matplotlib import pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels \
    import RBF, WhiteKernel, RationalQuadratic, ExpSineSquared
from utils.data import *

# get data
data = load_slip("data/slipNew.csv")
x = np.array(range(1,len(data)+1)).reshape(len(data),1)
y = data

x_train, y_train = x[:int(0.5*len(x))], y[:int(0.5*len(y))]
x_test, y_test = x[int(0.5*len(x)):], y[int(0.5*len(y)):]

# Kernel
k1 = ExpSineSquared(length_scale=10, periodicity=100) # long-term cycles
k2 = ExpSineSquared(length_scale=20.0, periodicity=20.0) # short-term oscillations
k3 = 0.5**2 * RationalQuadratic(length_scale=1.0, alpha=1.0)
k4 = 0.1**2 * RBF(length_scale=1) \
     + WhiteKernel(noise_level=0.1**2,
                   noise_level_bounds=(1e-3, 1e1,))  # noise
kernel = k3*(k1 + k2) + k4
gp = GaussianProcessRegressor(kernel=kernel, alpha=0,  n_restarts_optimizer=10, normalize_y=True, optimizer='fmin_l_bfgs_b')

# fit GP
gp.fit(x_train, y_train)
from IPython.display import display
display(gp)

# produce vector of values on x-axis
X_ = np.linspace(x.min(), x.max()+50, 1000)[:, np.newaxis]
# predict
y_pred, y_std = gp.predict(X_, return_std=True)

plt.rcParams.update({'font.size': 12})
plt.plot(x, y, c='b', label='Actual', linewidth=1) # true data
plt.scatter(x, y, c='b', label='Actual')
plt.plot(X_, y_pred, c='r', label='GP ($K_2$)', linewidth=1) # predictions
plt.fill_between(X_[:, 0], y_pred-2*y_std, y_pred+2*y_std,
                 alpha=0.3, color='k') # confidence intervals #C0C0C0
plt.ylim(-1.5, 1.5)
lo,hi = plt.ylim()
plt.plot([x[int(0.5*len(x))], x[int(0.5*len(x))]],[lo,hi],'k--')
plt.xlim(X_.min(), X_.max())
plt.xlabel("Time")
plt.ylabel("Slip Ratio")
plt.rcParams['axes.xmargin'] = 0
plt.rcParams['axes.ymargin'] = 0
plt.tight_layout()
plt.show()
