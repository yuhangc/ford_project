import numpy as np
from numpy.linalg import eig, inv


def fitEllipse(x, y):
    x = x[:,np.newaxis]
    y = y[:,np.newaxis]
    D = np.hstack((x*x, x*y, y*y, x, y, np.ones_like(x)))
    S = np.dot(D.T, D)
    C = np.zeros([6, 6])
    C[0, 2] = C[2, 0] = 2; C[1, 1] = -1
    E, V = eig(np.dot(inv(S), C))
    n = np.argmax(E)
    a = V[:, n]
    return a

def ellipse_center(a):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    num = b*b-a*c
    x0=(c*d-b*f)/num
    y0=(a*f-b*d)/num
    return np.array([x0,y0])


def ellipse_angle_of_rotation( a ):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    return 0.5*np.arctan(2*b/(a-c))


def ellipse_axis_length( a ):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    up = 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)
    down1=(b*b-a*c)*( (c-a)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
    down2=(b*b-a*c)*( (a-c)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
    res1=np.sqrt(up/down1)
    res2=np.sqrt(up/down2)
    return np.array([res1, res2])

def ellipse_angle_of_rotation2( a ):
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    if b == 0:
        if a > c:
            return 0
        else:
            return np.pi/2
    else: 
        if a > c:
            return np.arctan(2*b/(a-c))/2
        else:
            return np.pi/2 + np.arctan(2*b/(a-c))/2


def bound_param(param, lower_lim, upper_lim):
    if param < lower_lim:
        param = lower_lim
    elif param > upper_lim:
        param = upper_lim

    return param

def fit_ellipse(data, param_init, num_iter, alpha, delt_limit):
    xc = param_init[0]
    yc = param_init[1]
    theta = param_init[2]
    a = param_init[3]
    b = param_init[4]

    for iter in range(0, num_iter):
        for data_point in data:
            # calcualte stochastic gradient
            x = data_point[0]
            y = data_point[1]
            x_can = (x - xc) * np.cos(theta) + (y - yc) * np.sin(theta)
            y_can = -(x - xc) * np.sin(theta) + (y - yc) * np.cos(theta)

            d_xc = -2 * (x_can**2 / a**2 + y_can**2 / b**2 - 1) * (2.0 / a**2 * x_can * np.cos(theta)
                                                                   - 2.0 / b**2 * y_can * np.sin(theta))
            d_yc = -2 * (x_can**2 / a**2 + y_can**2 / b**2 - 1) * (2.0 / a**2 * x_can * np.sin(theta)
                                                                   + 2.0 / b**2 * y_can * np.cos(theta))
            d_theta = 2 * (x_can**2 / a**2 + y_can**2 / b**2 - 1) * (2.0 / a**2 * x_can * y_can
                                                                     - 2.0 / b**2 * y_can * x_can)

            # stochastic gradient descent
            xc -= alpha * d_xc * 100
            yc -= alpha * d_yc * 100
            theta -= alpha * d_theta

            # # bound the parameters
            # xc = bound_param(xc, param_init[0] - delt_limit[0], param_init[0] + delt_limit[0])
            # yc = bound_param(yc, param_init[1] - delt_limit[1], param_init[1] + delt_limit[1])
            # theta = bound_param(theta, param_init[2] - delt_limit[2], param_init[2] + delt_limit[2])

    return [xc, yc, theta, a, b]
