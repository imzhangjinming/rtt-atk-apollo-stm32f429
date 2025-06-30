import serial
import re
import time
import numpy as np
import csv,datetime
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz
from scipy.optimize import curve_fit 
from mpl_toolkits.mplot3d import Axes3D

# 配置串口参数 - 根据你的硬件修改这些值
PORT = 'COM13'       # Windows系统使用'COMx'，Linux/Mac使用'/dev/ttyUSB0'或'/dev/ttyAMA0'
BAUD_RATE = 115200  # 确保与STM32程序设置的波特率一致

ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
print(f"连接串口 {ser.name}...")

cal_size = 1000 # points to use for calibration

mag_labels = ['m_x','m_y','m_z'] # mag labels for plots
mag_cal_axes = ['z','y','x'] # axis order being rotated
cal_rot_indices = [[0,1],[1,2],[0,2]] # indices of heading for each axis

def parse_mpu9250_data(line):
    """
    解析符合格式的MPU9250数据行
    格式：ax,ay,az,gx,gy,gz,mx,my,mz
    """
    # 正则表达式匹配可能包含负号和小数点的数字
    pattern = r'-?\d+\.\d+|-?\d+'  # 匹配整数或浮点数（含负号）
    numbers = re.findall(pattern, line)
    
    if len(numbers) == 9:
        try:
            # 将字符串转换为浮点数
            data = [float(num) for num in numbers]
            return (
                data[0], data[1], data[2],  # ax, ay, az
                data[3], data[4], data[5],  # gx, gy, gz
                data[6], data[7], data[8]   # mx, my, mz
            )
        except ValueError:
            pass  # 转换失败则忽略
    return None

def read_mpu9250_data():
    """
    读取MPU9250数据并返回一个包含加速度、陀螺仪和磁力计数据的元组
    """
    # 读取一行数据（确保STM32发送时使用换行符）
    not_ready = True
    while(not_ready):
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if line:
            # 尝试解析数据
            parsed = parse_mpu9250_data(line)
            if parsed:
                ax, ay, az, gx, gy, gz, mx, my, mz = parsed
                print(f"加速度: ({ax:.2f}, {ay:.2f}, {az:.2f}) | "
                    f"陀螺仪: ({gx:.2f}, {gy:.2f}, {gz:.2f}) | "
                    f"磁力计: ({mx:.2f}, {my:.2f}, {mz:.2f})")
                not_ready = False
            # else: # 调试时可查看被忽略的行
            #     print(f"忽略无效数据: {line}")

    return ax, ay, az, gx, gy, gz, mx, my, mz

def get_gyro():
    _,_,_,wx,wy,wz,_,_,_ = read_mpu9250_data() # read and convert gyro data
    return wx,wy,wz

def gyro_cal():
    print("-"*50)
    print('Gyro Calibrating - Keep the IMU Steady')
    [get_gyro() for ii in range(0,cal_size)] # clear buffer before calibration
    mpu_array = []
    gyro_offsets = [0.0,0.0,0.0]
    while True:
        try:
            wx,wy,wz = get_gyro() # get gyro vals
        except:
            continue

        mpu_array.append([wx,wy,wz])

        if np.shape(mpu_array)[0]==cal_size:
            for qq in range(0,3):
                gyro_offsets[qq] = np.mean(np.array(mpu_array)[:,qq]) # average
            break
    print('Gyro Calibration Complete')
    return gyro_offsets

def mpu9250_gyro_calibration():
    #
    ###################################
    # Gyroscope Offset Calculation
    ###################################
    #
    gyro_labels = ['w_x','w_y','w_z'] # gyro labels for plots
    gyro_offsets = gyro_cal() # calculate gyro offsets
    #
    ###################################
    # Record new data 
    ###################################
    #
    data = np.array([get_gyro() for ii in range(0,cal_size)]) # new values
    #
    ###################################
    # Plot with and without offsets
    ###################################
    #
    plt.style.use('ggplot')
    fig,axs = plt.subplots(2,1,figsize=(12,9))
    for ii in range(0,3):
        axs[0].plot(data[:,ii],
                    label='${}$, Uncalibrated'.format(gyro_labels[ii]))
        axs[1].plot(data[:,ii]-gyro_offsets[ii],
                    label='${}$, Calibrated'.format(gyro_labels[ii]))
    axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$w_{x,y,z}$ [$^{\circ}/s$]',fontsize=18)
    axs[1].set_ylabel('$w_{x,y,z}$ [$^{\circ}/s$]',fontsize=18)
    axs[1].set_xlabel('Sample',fontsize=18)
    axs[0].set_ylim([-50,50]);axs[1].set_ylim([-20,20])
    axs[0].set_title('Gyroscope Calibration Offset Correction\n Gyro offsets: ({:.2f},{:.2f},{:.2f})'.format(gyro_offsets[0],gyro_offsets[1],gyro_offsets[2]),fontsize=22)
    fig.savefig('applications/mpu9250/calibration/gyro_calibration_output.png',dpi=300,
                bbox_inches='tight',facecolor='#FCFCFC')
    fig.show()

def mpu9250_gyro_calibration_test():
    #
    ###################################
    # Gyroscope Offset Calculation
    ###################################
    #
    gyro_labels = ['\omega_x','\omega_y','\omega_z'] # gyro labels for plots
    gyro_offsets = gyro_cal() # calculate gyro offsets
    #
    ###################################
    # Record new data 
    ###################################
    #
    input("Press Enter and Rotate Gyro 360 degrees")
    print("Recording Data...")
    record_time = 5 # how long to record
    data,t_vec = [],[]
    t0 = time.time()
    while time.time()-t0<record_time:
        data.append(get_gyro())
        t_vec.append(time.time()-t0)
    samp_rate = np.shape(data)[0]/(t_vec[-1]-t_vec[0]) # sample rate
    print("Stopped Recording\nSample Rate: {0:2.0f} Hz".format(samp_rate))
    #
    ##################################
    # Offset and Integration of gyro
    # and plotting results
    ##################################
    #
    rot_axis = 2 # axis being rotated (2 = z-axis)
    data_offseted = np.array(data)[:,rot_axis]-gyro_offsets[rot_axis]
    integ1_array = cumtrapz(data_offseted,x=t_vec) # integrate once in time
    #
    # print out reuslts
    print("Integration of {} in {}".format(gyro_labels[rot_axis],
                    gyro_labels[rot_axis].split("_")[1])+\
            "-dir: {0:2.2f}m".format(integ1_array[-1]))
    #
    # plotting routine
    plt.style.use('ggplot')
    fig,axs = plt.subplots(2,1,figsize=(12,9))
    axs[0].plot(t_vec,data_offseted,label="$"+gyro_labels[rot_axis]+"$")
    axs[1].plot(t_vec[1:],integ1_array,
                label=r"$\theta_"+gyro_labels[rot_axis].split("_")[1]+"$")
    [axs[ii].legend(fontsize=16) for ii in range(0,len(axs))]
    axs[0].set_ylabel('Angular Velocity, $\omega_{}$ [$^\circ/s$]'.format(gyro_labels[rot_axis].\
                                        split("_")[1]),fontsize=16)
    axs[1].set_ylabel(r'Rotation, $\theta_{}$ [$^\circ$]'.format(gyro_labels[rot_axis].\
                                            split("_")[1]),fontsize=16)
    axs[1].set_xlabel('Time [s]',fontsize=16)
    axs[0].set_title('Gyroscope Integration over 180$^\circ$ Rotation',
                        fontsize=18)
    fig.savefig('applications/mpu9250/calibration/gyroscope_integration_180deg_rot.png',dpi=300,
                bbox_inches='tight',facecolor='#FFFFFF')
    plt.show()

def accel_fit(x_input,m_x,b):
    return (m_x*x_input)+b # fit equation for accel calibration

def get_accel():
    ax,ay,az,_,_,_,_,_,_ = read_mpu9250_data() # read and convert accel data
    return ax/1000.0, ay/1000.0, az/1000.0


def accel_cal():
    print("-"*50)
    print("Accelerometer Calibration")
    mpu_offsets = [[],[],[]] # offset array to be printed
    axis_vec = ['z','y','x'] # axis labels
    cal_directions = ["upward","downward","perpendicular to gravity"] # direction for IMU cal
    cal_indices = [2,1,0] # axis indices
    for qq,ax_qq in enumerate(axis_vec):
        ax_offsets = [[],[],[]]
        print("-"*50)
        for direc_ii,direc in enumerate(cal_directions):
            input("-"*8+" Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -"+\
              ax_qq+"-axis pointed "+direc)
            [read_mpu9250_data() for ii in range(0,2000)] # clear buffer between readings
            mpu_array = []
            while len(mpu_array)<cal_size:
                try:
                    ax,ay,az = get_accel()
                    mpu_array.append([ax,ay,az]) # append to array
                except:
                    continue
            ax_offsets[direc_ii] = np.array(mpu_array)[:,cal_indices[qq]] # offsets for direction

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        popts,_ = curve_fit(accel_fit,np.append(np.append(ax_offsets[0],
                                 ax_offsets[1]),ax_offsets[2]),
                   np.append(np.append(1.0*np.ones(np.shape(ax_offsets[0])),
                    -1.0*np.ones(np.shape(ax_offsets[1]))),
                        0.0*np.ones(np.shape(ax_offsets[2]))),
                            maxfev=10000)
        mpu_offsets[cal_indices[qq]] = popts # place slope and intercept in offset array
    print('Accelerometer Calibrations Complete')
    print("Accelerometer Offsets:")
    print(mpu_offsets)
    return mpu_offsets

def mpu9250_accel_calibration():
    #
    ###################################
    # Accelerometer Gravity Calibration
    ###################################
    #
    accel_labels = ['a_x','a_y','a_z'] # gyro labels for plots
    accel_coeffs = accel_cal() # grab accel coefficients
#
    ###################################
    # Record new data 
    ###################################
    #
    data = np.array([get_accel() for ii in range(0,cal_size)]) # new values


    print("Accelerometer Offsets:")
    print(accel_coeffs)
    #
    ###################################
    # Plot with and without offsets
    ###################################
    #
    plt.style.use('ggplot')
    fig,axs = plt.subplots(2,1,figsize=(12,9))
    for ii in range(0,3):
        axs[0].plot(data[:,ii],
                    label='${}$, Uncalibrated'.format(accel_labels[ii]))
        axs[1].plot(accel_fit(data[:,ii],*accel_coeffs[ii]),
                    label='${}$, Calibrated'.format(accel_labels[ii]))
    axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
    axs[1].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
    axs[1].set_xlabel('Sample',fontsize=18)
    axs[0].set_ylim([-2,2]);axs[1].set_ylim([-2,2])
    axs[0].set_title('Accelerometer Calibration Calibration Correction\nAccel Calibration Offsets: ',fontsize=18)
    fig.savefig('applications/mpu9250/calibration/accel_calibration_output.png',dpi=300,
                bbox_inches='tight',facecolor='#FCFCFC')
    fig.show()

def outlier_removal(x_ii,y_ii,z_ii):
    x_diff = np.append(0.0,np.diff(x_ii)) # looking for outliers
    stdev_amt = 5.0 # standard deviation multiplier
    x_outliers = np.where(np.abs(x_diff)>np.abs(np.mean(x_diff))+\
                          (stdev_amt*np.std(x_diff)))
    # x_inliers  = np.where(np.abs(x_diff)<np.abs(np.mean(x_diff))+\
    #                       (stdev_amt*np.std(x_diff)))
    y_diff     = np.append(0.0,np.diff(y_ii)) # looking for outliers
    y_outliers = np.where(np.abs(y_diff)>np.abs(np.mean(y_diff))+\
                          (stdev_amt*np.std(y_diff)))
    # y_inliers  = np.where(np.abs(y_diff)<np.abs(np.mean(y_diff))+\
    #              (stdev_amt*np.std(y_diff)))
    z_diff     = np.append(0.0,np.diff(z_ii)) # looking for outliers
    z_outliers = np.where(np.abs(z_diff)>np.abs(np.mean(z_diff))+\
                          (stdev_amt*np.std(z_diff)))
    # z_inliers  = np.where(np.abs(z_diff)<np.abs(np.mean(z_diff))+\
    #              (stdev_amt*np.std(z_diff)))
    
    if len(x_outliers)!=0:
        x_ii[x_outliers] = np.nan # null outlier
        y_ii[x_outliers] = np.nan # null outlier
        z_ii[x_outliers] = np.nan # null outlier
    if len(y_outliers)!=0:
        y_ii[y_outliers] = np.nan # null outlier
        x_ii[y_outliers] = np.nan # null outlier
        z_ii[y_outliers] = np.nan # null outlier
    if len(z_outliers)!=0:
        z_ii[z_outliers] = np.nan # null outlier
        x_ii[z_outliers] = np.nan # null outlier
        y_ii[z_outliers] = np.nan # null outlier
    # find common value in inliers
    x_inliers  = np.where(np.isnan(x_ii)==False)
    y_inliers  = np.where(np.isnan(y_ii)==False)
    z_inliers  = np.where(np.isnan(z_ii)==False)

    x_ii = x_ii[x_inliers]
    y_ii = y_ii[y_inliers]
    z_ii = z_ii[z_inliers]

    return x_ii,y_ii,z_ii

def mag_cal():
    print("-"*50)
    print("Magnetometer Calibration")
    mag_cal_rotation_vec = [] # variable for calibration calculations
    for qq,ax_qq in enumerate(mag_cal_axes):
        input("-"*8+" Press Enter and Start Rotating the IMU Around the "+ax_qq+"-axis")
        print("\t When Finished, Press CTRL+C")
        mag_array = []
        t0 = time.time()
        while True:
            try:
                _,_,_,_,_,_,mx,my,mz = read_mpu9250_data() # read and convert AK8963 magnetometer data
            except KeyboardInterrupt:
                break
            except:
                continue
            # mx, my, mz = outlier_removal(mx,my,mz)
            mag_array.append([mx,my,mz]) # mag array
        mag_array = np.array(mag_array) # make numpy array
        mx, my, mz = outlier_removal(mag_array[:,0],mag_array[:,1],mag_array[:,2]) # remove outliers
        mag_cal_rotation_vec.append(np.concatenate((mx.reshape(-1,1),my.reshape(-1,1),mz.reshape(-1,1)),axis=1)) # concatenate all data

    
    mag_cal_rotation_vec = np.concatenate((mag_cal_rotation_vec[0],mag_cal_rotation_vec[1],mag_cal_rotation_vec[2]), axis=0)

    Y = - np.square(mag_cal_rotation_vec[:,0])
    Psi = np.concatenate((  np.square(mag_cal_rotation_vec[:, 1]).reshape(-1, 1),
                            np.square(mag_cal_rotation_vec[:, 2]).reshape(-1, 1),
                            mag_cal_rotation_vec[:, 0].reshape(-1, 1),
                            mag_cal_rotation_vec[:, 1].reshape(-1, 1),
                            mag_cal_rotation_vec[:, 2].reshape(-1, 1),
                            np.ones(np.shape(mag_cal_rotation_vec[:, 0])).reshape(-1, 1)),
                            axis=1) 
    
    P = np.linalg.inv(np.matmul(Psi.T,Psi))
    Theta = np.matmul(P,np.matmul(Psi.T,Y))

    x_c = Theta[2] / (-2)
    y_c = Theta[3] / (-2 * Theta[0])
    z_c = Theta[4] / (-2 * Theta[1])
    x_r = np.sqrt(x_c**2 + Theta[0]*y_c**2 + Theta[1]*z_c**2 - Theta[5])
    y_r = np.sqrt(x_r**2 / Theta[0])
    z_r = np.sqrt(x_r**2 / Theta[1])

    ak_fit_coeffs = [x_c,y_c,z_c,x_r,y_r,z_r] 

    return ak_fit_coeffs,mag_cal_rotation_vec

def mag_cal_plot(mag_coeffs, mag_cal_rotation_vec):
    plt.style.use('ggplot') # start figure
    fig,axs = plt.subplots(1,2,figsize=(12,7)) # start figure

    x_c, y_c, z_c, x_r, y_r, z_r = mag_coeffs # grab mag coefficients

    # plot ellipsoid in axs[0], its center is at (x_c,y_c,z_c) and its axes are aligned with the global coordinate system
    # half axes lengths are x_r, y_r, z_r
    # plot 3d scatter in axs[0], x,y,z are in mag_cal_rotation_vec

    # Create a meshgrid for the ellipsoid plot
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    u, v = np.meshgrid(u, v)

    x = x_c + x_r * np.cos(u) * np.sin(v)
    y = y_c + y_r * np.sin(u) * np.sin(v)
    z = z_c + z_r * np.cos(v)

    x_scatter, y_scatter, z_scatter = mag_cal_rotation_vec[:, 0], mag_cal_rotation_vec[:, 1], mag_cal_rotation_vec[:, 2]

    # Plot the ellipsoid in the first subplot (axs[0])
    ax1 = axs[0]  # First subplot
    ax1 = fig.add_subplot(121, projection='3d')  # Set it to 3D plot
    ax1.plot_surface(x, y, z, color='b', alpha=0.5)  # Plot the ellipsoid surface
    ax1.scatter(x_scatter, y_scatter, z_scatter, color='r', label='Magnetic Raw Points')  # Plot scatter
    ax1.set_xlabel('X-axis')
    ax1.set_ylabel('Y-axis')
    ax1.set_zlabel('Z-axis')
    ax1.set_title('Ellipsoid and Magnetic Raw Points')
    ax1.legend()

    x_scatter_after_calib = 1 / x_r * (x_scatter - x_c) # x after calibration
    y_scatter_after_calib = 1 / y_r * (y_scatter - y_c) # y after calibration
    z_scatter_after_calib = 1 / z_r * (z_scatter - z_c) # z after calibration

    x_uint = 0 + 1.0 * np.cos(u) * np.sin(v)
    y_uint = 0 + 1.0 * np.sin(u) * np.sin(v)
    z_uint = 0 + 1.0 * np.cos(v)

    ax2 = axs[1]  # First subplot
    ax2 = fig.add_subplot(122, projection='3d')  # Set it to 3D plot
    ax2.plot_surface(x_uint, y_uint, z_uint, color='b', alpha=0.5)  # Plot the ellipsoid surface
    ax2.scatter(x_scatter_after_calib, y_scatter_after_calib, z_scatter_after_calib, color='r', label='Magnetic Calibrated Points')  # Plot scatter
    ax2.set_xlabel('X-axis')
    ax2.set_ylabel('Y-axis')
    ax2.set_zlabel('Z-axis')
    ax2.set_title('Unit Sphere and Magnetic Calibrated Points')
    ax2.legend()

    # mag_lims = [np.nanmin(np.nanmin(mag_cal_rotation_vec)),
    #             np.nanmax(np.nanmax(mag_cal_rotation_vec))] # array limits
    # mag_lims = [-1.1*np.max(mag_lims),1.1*np.max(mag_lims)] # axes limits
    # for jj in range(0,2):
    #     axs[jj].set_ylim(mag_lims) # set limits
    #     axs[jj].set_xlim(mag_lims) # set limits
    #     axs[jj].legend() # legend
    #     axs[jj].set_aspect('equal',adjustable='box') # square axes
    fig.savefig('applications/mpu9250/calibration/mag_cal_hard_offset_white.png',dpi=300,bbox_inches='tight',
                facecolor='#FFFFFF') # save figure
    plt.show() #show plot
    print(1)

def mpu9250_mag_calibration():
    #
    ###################################
    # Magnetometer Calibration
    ###################################
    #
    mag_coeffs,mag_cal_rotation_vec = mag_cal() # grab mag coefficients
    #
    ###################################
    # Plot with and without offsets
    ###################################
    #
    mag_cal_plot(mag_coeffs,mag_cal_rotation_vec) # plot un-calibrated and calibrated results
    #
    print("Magnetometer Offsets:")
    print(mag_coeffs)
    

if __name__ == '__main__':
    # mpu9250_gyro_calibration()
    # mpu9250_gyro_calibration_test()
    # mpu9250_accel_calibration()
    mpu9250_mag_calibration()
    
    
    # while True:
    #     read_mpu9250_data()
    
    ser.close()