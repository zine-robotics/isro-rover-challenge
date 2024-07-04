import scipy
import numpy as np
from scipy.interpolate import griddata
import math 

x_axis_real = [0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 15, 15, 15, 15, 15, 15, 15, 15, 15, 20, 20, 20, 20, 20, 20, 20, 20, 20]
y_axis_real = [-20, -15, -10, -5, 0, 5, 10, 15, 20, -20, -15, -10, -5, 0, 5, 10, 15, 20, -20, -15, -10, -5, 0, 5, 10, 15, 20, -20, -15, -10, -5, 0, 5, 10, 15, 20, -20, -15, -10, -5, 0, 5, 10, 15, 20]
x_axis_predicted = [53, 53, 52, 52, 49, 49, 53, 53, 53, 63, 65, 65, 66, 56, 54, 57, 64, 63, 70, 71, 71, 70, 59, 60, 64, 69, 73, 80, 80, 80, 79, 65, 65, 65, 79, 79, 90, 92, 90, 89, 70, 69, 75, 85, 87]
y_axis_predicted = [-35, -26, -17, -10, -1, 5, 15, 25, 34, -39, -28, -20, -11, -1, 4, 13, 26, 35, -39, -29, -19, -10, -1, 6, 13, 29, 42, -44, -30, -21, -12, -3, 6, 13, 27, 38, -43, -31, -19, -11, 0, 9, 13, 30, 43]

x_axis_real = np.array(x_axis_real)
x_axis_predicted = np.array(x_axis_predicted)
x_axis_real = np.add(x_axis_real, 50)
# x_axis_predicted = np.subtract(x_axis_predicted, 50)
x_axis_error = np.subtract(x_axis_real, x_axis_predicted)

y_axis_real = np.array(y_axis_real)
y_axis_predicted = np.array(y_axis_predicted)
y_axis_error = np.subtract(y_axis_real, y_axis_predicted)

# Create a grid of points for interpolation
# grid_x, grid_y = np.meshgrid(np.linspace(min(x_axis_predicted)-10, max(x_axis_predicted)+10, 500),
#                              np.linspace(min(y_axis_predicted)-10, max(y_axis_predicted)+10, 500))

def error_corrector(view_angle,pred_x, pred_y,pred_z):

    pred_x = math.cos(math.radians(view_angle))*pred_x
    pred_z = math.cos(math.radians(view_angle))*pred_z - math.sin(math.radians(view_angle))*pred_x


    # Create a grid of points for interpolation
    grid_x, grid_y = np.meshgrid(np.linspace(min(x_axis_predicted)-10, max(x_axis_predicted)+10, 500),
                                 np.linspace(min(y_axis_predicted)-10, max(y_axis_predicted)+10, 500))

    # Interpolate the error for the given predicted point
    x_error_at_pred = griddata((x_axis_predicted, y_axis_predicted), x_axis_error, (pred_x, pred_y), method='cubic')
    y_error_at_pred = griddata((x_axis_predicted, y_axis_predicted), y_axis_error, (pred_x, pred_y), method='cubic')

    if np.isnan(x_error_at_pred):
        x_error_at_pred = griddata((x_axis_predicted, y_axis_predicted), x_axis_error, (pred_x, pred_y), method='nearest')
    if np.isnan(y_error_at_pred):
        y_error_at_pred = griddata((x_axis_predicted, y_axis_predicted), y_axis_error, (pred_x, pred_y), method='nearest')

    # Calculate the interpolated real values by adjusting the predicted values with the interpolated error
    real_x_at_pred = pred_x + x_error_at_pred
    real_y_at_pred = pred_y + y_error_at_pred

    return pred_x, real_y_at_pred, pred_z

# x,y,z = error_corrector(0,67,-36,-14)
# print(x,y,z)