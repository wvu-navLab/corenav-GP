import numpy as np

from matplotlib import pyplot as plt

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels \
    import RBF, WhiteKernel, RationalQuadratic, ExpSineSquared
from utils.data import *

# get data
data = load_slip("data/slipData9.csv")
x = np.array(range(1,len(data)+1)).reshape(len(data),1)
y = data
x_train, y_train = x[:int(0.7*len(x))], y[:int(0.7*len(y))]
x_test, y_test = x[int(0.7*len(x)):], y[int(0.7*len(y)):]

# Kernel
k1 = ExpSineSquared(length_scale=10, periodicity=100) # long-term cycles
k2 = ExpSineSquared(length_scale=20.0, periodicity=20.0) # short-term oscillations
k3 = 0.5**2 * RationalQuadratic(length_scale=1.0, alpha=1.0)
k4 = 0.1**2 * RBF(length_scale=0.1) \
     + WhiteKernel(noise_level=0.1**2,
                   noise_level_bounds=(1e-3, np.inf))  # noise
kernel = k3+(k1 + k2) + k4

# k1 = 66.0**2 * RBF(length_scale=67.0)  # long term smooth rising trend
# k2 = 2.4**2 * RBF(length_scale=90.0) \
#     * ExpSineSquared(length_scale=1.3, periodicity=1.0)  # seasonal component
# # medium term irregularity
# k3 = 0.66**2 \
#     * RationalQuadratic(length_scale=1.2, alpha=0.78)
# k4 = 0.18**2 * RBF(length_scale=0.134) \
#     + WhiteKernel(noise_level=0.19**2)  # noise terms
# kernel = k1 + k2 + k3 + k4

# k1 = 50.0**2 * RBF(length_scale=50.0)  # long term smooth rising trend
# k2 = 2.0**2 * RBF(length_scale=100.0) \
#     * ExpSineSquared(length_scale=1.0, periodicity=1.0,
#                      periodicity_bounds="fixed")  # seasonal component
# # medium term irregularities
# k3 = 0.5**2 * RationalQuadratic(length_scale=1.0, alpha=1.0)
# k4 = 0.1**2 * RBF(length_scale=0.1) \
#     + WhiteKernel(noise_level=0.1**2,
#                   noise_level_bounds=(1e-3, np.inf))  # noise terms
# kernel = k1 + k2 + k3 + k4

gp = GaussianProcessRegressor(kernel=kernel, alpha=0, normalize_y=True)

# fit GP
gp.fit(x_train, y_train)
# produce vector of values on x-axis
X_ = np.linspace(x.min(), x.max()+50, 1000)[:, np.newaxis]
# predict
y_pred, y_std = gp.predict(X_, return_std=True)

# figure
# plt.figure(num=None, figsize=(4,4), dpi=300)
plt.rcParams.update({'font.size': 12})
plt.plot(x, y, c='b', label='Actual', linewidth=1) # true data
plt.scatter(x, y, c='b', label='Actual')
plt.plot(X_, y_pred, c='r', label='GP ($K_2$)', linewidth=1) # predictions
plt.fill_between(X_[:, 0], y_pred-2*y_std, y_pred+2*y_std,
                 alpha=0.3, color='k') # confidence intervals #C0C0C0
plt.ylim(-1.5, 1.5)
lo,hi = plt.ylim()
plt.plot([x[int(0.7*len(x))], x[int(0.7*len(x))]],[lo,hi],'k--')
plt.xlim(X_.min(), X_.max())
plt.xlabel("Time")
plt.ylabel("Slip Ratio")
plt.rcParams['axes.xmargin'] = 0
plt.rcParams['axes.ymargin'] = 0
plt.tight_layout()
plt.show()

#y_pred, y_std = gp.predict(x, return_std=True)
#mse = np.mean((y_test-y_pred[int(0.7*len(x)):])**2)
