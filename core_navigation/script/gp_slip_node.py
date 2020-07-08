#!/usr/bin/env python
import sys
sys.path.append('/home/urcnuc115/GPy')
import GPy
import rospy
import numpy as np
# from matplotlib import pyplot as plt
#from utils.data import *
from core_nav.msg import GP_Input
from core_nav.msg import GP_Output

pub = rospy.Publisher('/core_nav/core_nav/gp_result', GP_Output, queue_size=1)

def callback(data):
    # Do GP stuff here *********

    X = data.time_array
    #X = [1,2,3,4,5]
    X = np.array(X).reshape(len(X),1)

    Y = data.slip_array
    #Y = [0.1,0.05,-0.009,0.1,-0.002]
    Y = np.array(Y).reshape(len(Y),1)

    per = 0.9

    x_train, y_train = X[:int(per*len(X))], Y[:int(per*len(Y))]
    x_test, y_test = X[int(per*len(X)):], Y[int(per*len(Y)):]
    kernel=(GPy.kern.RBF(1))*GPy.kern.Brownian(1)
# # GPy.kern.Matern32(1), GPy.kern.Matern52(1), GPy.kern.Brownian(1),
# # GPy.kern.Bias(1), GPy.kern.Linear(1), GPy.kern.PeriodicExponential(1),
# # GPy.kern.White(1)]
    m = GPy.models.GPRegression(x_train,y_train,kernel)
    m.optimize(messages=False)

# from IPython.display import display
# display(m)
    means= [] #predictions
    variances = [] #uncertainty
    # means2= [] #predictions
    # variances2 = [] #uncertainty
# X_=np.arange(X.min(),X.max()+50, 0.1)
    X_=np.arange(X.min(),X.max()+600, 1)

    for x in X_:
        mean,covar = m.predict(np.array([[x]]))
        variances.append(covar[0])
        means.append(mean[0])
        # means2.append(mean[0])
        # variances2.append(covar[0])


    # **************************

    # Write output arrays into msg_out.mean and msg_out.sigma
    msg_out = GP_Output()
    # TODO: need to populate the header (i.e. msg_out.header)
    msg_out.mean = np.array(means[int(len(X)):]) #[0,1,3,4] # for example
    msg_out.sigma = 2*np.sqrt(np.array(variances[int(len(X)):])) #[6,7,8,9] # for example
    pub.publish(msg_out)

    # ses = 2*np.sqrt(np.array(variances2))
    # means = np.array(means2)
    # var1=np.array(means+ses).reshape(1,len(means+ses))
    # var2=np.array(means-ses).reshape(1,len(means-ses))
    # m.plot()
    # plt.xlim(X.min(), X_.max())
    # plt.plot(X,Y, c='r',linewidth=0.2)
    # plt.fill_between(X_, var2[0,:], var1[0,:], alpha=0.3, color='k') # confidence intervals #C0C0C0
    # plt.plot(x_test, y_test,'xk')
    # plt.ylim(-1.5, 1.5)
    # plt.show()


def gaussian_process():
    rospy.init_node('gp_slip_node')
    rospy.Subscriber('/core_nav/core_nav/gp_input', GP_Input, callback)
    rospy.spin()

if __name__ == '__main__':
    gaussian_process()
