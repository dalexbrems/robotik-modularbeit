import csv
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


class Plot():
    def __init__(self):
        pass

    def to_pd(self, filename, has_header=False):
        data = pd.read_csv(filename, sep=',', header=has_header)
        return data

    def add_headers(self, filename):
        names = ['t', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
                 'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6',
                 'x', 'y', 'z', 'rx','ry', 'rz',
                 'sp1', 'sp2', 'sp3', 'sp4', 'sp5', 'sp6',
                 'qt1', 'qt2', 'qt3', 'qt4', 'qt5', 'qt6',
                 'qdt1', 'qdt2', 'qdt3', 'qdt4', 'qdt5', 'qdt6',
                 'qddt1', 'qddt2', 'qddt3', 'qddt4', 'qddt5', 'qddt6']

        data = pd.read_csv(filename, sep=',', header=None)

        if data[0][0] == 't':
            print('File already has headers.')
            return

        data.columns = names
        new_f_name = filename[:-4]
        data.to_csv(new_f_name + '_named.csv', sep=',', index=False)

    def plot_df(self, frame, columns):
        print(frame)
        plt.figure(1)
        frame.plot(x='t', y=[columns])

    def rad_2_deg(self):
        pass

    def deg_2_rad(self):
        pass

def plot_data(filename, selector=('q', 'qd', 'pose', 'tcp_v', 'qt', 'qdt', 'qddt'), in_degrees=False):

    t = []

    q1 = []
    q2 = []
    q3 = []
    q4 = []
    q5 = []
    q6 = []

    qd1 = []
    qd2 = []
    qd3 = []
    qd4 = []
    qd5 = []
    qd6 = []

    x = []
    y = []
    z = []

    rx = []
    ry = []
    rz = []

    sp1 = []
    sp2 = []
    sp3 = []
    sp4 = []
    sp5 = []
    sp6 = []
    
    qt1 = []
    qt2 = []
    qt3 = []
    qt4 = []
    qt5 = []
    qt6 = []
    
    qdt1 = []
    qdt2 = []
    qdt3 = []
    qdt4 = []
    qdt5 = []
    qdt6 = []
    
    qddt1 = []
    qddt2 = []
    qddt3 = []
    qddt4 = []
    qddt5 = []
    qddt6 = []


    with open(filename, 'r') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
        for row in reader:
            t.append(row[0])

            q1.append(row[1])
            q2.append(row[2])
            q3.append(row[3])
            q4.append(row[4])
            q5.append(row[5])
            q6.append(row[6])

            qd1.append(row[7])
            qd2.append(row[8])
            qd3.append(row[9])
            qd4.append(row[10])
            qd5.append(row[11])
            qd6.append(row[12])

            x.append(row[13])
            y.append(row[14])
            z.append(row[15])
            rx.append(row[16])
            ry.append(row[17])
            rz.append(row[18])

            sp1.append(row[19])
            sp2.append(row[20])
            sp3.append(row[21])
            sp4.append(row[22])
            sp5.append(row[23])
            sp6.append(row[24])
            
            qt1.append(row[25]) 
            qt2.append(row[26]) 
            qt3.append(row[27]) 
            qt4.append(row[28])
            qt5.append(row[29])
            qt6.append(row[30])
            
            qdt1.append(row[31]) 
            qdt2.append(row[32]) 
            qdt3.append(row[33]) 
            qdt4.append(row[34])
            qdt5.append(row[35])
            qdt6.append(row[36])
            
            qddt1.append(row[37]) 
            qddt2.append(row[38]) 
            qddt3.append(row[39]) 
            qddt4.append(row[40])
            qddt5.append(row[41])
            qddt6.append(row[42])


    fig, axs = plt.subplots(8, sharex='all', figsize=(12, 8))
    fig.suptitle(f'Robot Data "{filename}"')

    fig = plt.figure(1)

    if 'q' in selector:
        for i, q in enumerate([q1, q2, q3, q4, q5, q6]):
            if in_degrees:
                q = np.rad2deg(q)
                #print(q)
            axs[0].plot(t, q, label=f'q{i+1}')

    if 'qd' in selector:
        for i, qd in enumerate([qd1, qd2, qd3, qd4, qd5, qd6]):
            if in_degrees:
                qd = np.rad2deg(qd)
            axs[1].plot(t, qd, label=f'qd{i+1}')

    if 'pose' in selector:
        for i, tcp_coords in zip(['x', 'y', 'z'], [x, y, z]):
            axs[2].plot(t, tcp_coords, label=f'{i}')

        for i, tcp_orient in zip(['rx', 'ry', 'rz'], [rx, ry, rz]):
            if in_degrees:
                tcp_orient = np.rad2deg(tcp_orient)

            axs[3].plot(t, tcp_orient, label=f'{i}')

    if 'tcp_v' in selector:
        for i, speed in enumerate([sp1, sp2, sp3, sp4, sp5, sp6]):
            axs[4].plot(t, speed, label=f'vel tcp{i+1}')

    # plot accelerations
    if 'qdd' in selector:
        for i, speed in enumerate([sp1, sp2, sp3, sp4, sp5, sp6]):
            print('Zeitticks: ', len(t))

    if 'qt' in selector:
        for i, qt in enumerate([qt1, qt2, qt3, qt4, qt5, qt6]):
            if in_degrees:
                qt = np.rad2deg(qt)
            axs[5].plot(t, qt, label=f'qt{i+1}')

    if 'qdt' in selector:
        for i, qdt in enumerate([qdt1, qdt2, qdt3, qdt4, qdt5, qdt6]):
            if in_degrees:
                qdt = np.rad2deg(qdt)
            axs[6].plot(t, qdt, label=f'qdt{i+1}')

    if 'qddt' in selector:
        for i, qddt in enumerate([qddt1, qddt2, qddt3, qddt4, qddt5, qddt6]):
            if in_degrees:
                qddt = np.rad2deg(qddt)
            axs[7].plot(t, qddt, label=f'qddt{i+1}')


    """
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    axs[3].legend()
    axs[4].legend()
    axs[5].legend()
    axs[6].legend()
    axs[7].legend()

    axs[0].grid()
    axs[1].grid()
    axs[2].grid()
    axs[3].grid()
    axs[4].grid()
    axs[5].grid()
    axs[6].grid()
    axs[7].grid()
    """

    plt.locator_params(axis='x', nbins=50)
    # axs[2].locator_params(axis='y', nbins=10)
    plt.xticks(rotation=25)
    plt.xlabel('System Time')

    plt.legend()
    plt.show()

def plot_data_single_windows(filename, in_degrees=False):
    t = []

    q1 = []
    q2 = []
    q3 = []
    q4 = []
    q5 = []
    q6 = []

    qd1 = []
    qd2 = []
    qd3 = []
    qd4 = []
    qd5 = []
    qd6 = []

    x = []
    y = []
    z = []

    rx = []
    ry = []
    rz = []

    sp1 = []
    sp2 = []
    sp3 = []
    sp4 = []
    sp5 = []
    sp6 = []

    qt1 = []
    qt2 = []
    qt3 = []
    qt4 = []
    qt5 = []
    qt6 = []

    qdt1 = []
    qdt2 = []
    qdt3 = []
    qdt4 = []
    qdt5 = []
    qdt6 = []

    qddt1 = []
    qddt2 = []
    qddt3 = []
    qddt4 = []
    qddt5 = []
    qddt6 = []


    with open(filename, 'r') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
        for row in reader:
            t.append(row[0])

            q1.append(row[1])
            q2.append(row[2])
            q3.append(row[3])
            q4.append(row[4])
            q5.append(row[5])
            q6.append(row[6])

            qd1.append(row[7])
            qd2.append(row[8])
            qd3.append(row[9])
            qd4.append(row[10])
            qd5.append(row[11])
            qd6.append(row[12])

            x.append(row[13])
            y.append(row[14])
            z.append(row[15])
            rx.append(row[16])
            ry.append(row[17])
            rz.append(row[18])

            sp1.append(row[19])
            sp2.append(row[20])
            sp3.append(row[21])
            sp4.append(row[22])
            sp5.append(row[23])
            sp6.append(row[24])

            qt1.append(row[25])
            qt2.append(row[26])
            qt3.append(row[27])
            qt4.append(row[28])
            qt5.append(row[29])
            qt6.append(row[30])

            qdt1.append(row[31])
            qdt2.append(row[32])
            qdt3.append(row[33])
            qdt4.append(row[34])
            qdt5.append(row[35])
            qdt6.append(row[36])

            qddt1.append(row[37])
            qddt2.append(row[38])
            qddt3.append(row[39])
            qddt4.append(row[40])
            qddt5.append(row[41])
            qddt6.append(row[42])

    # fig, axs = plt.subplots(8, sharex='all', figsize=(12, 8))
    # fig.suptitle(f'Robot Data "{filename}"')


    for i, q in enumerate([q1, q2, q3, q4, q5, q6]):
        if in_degrees:
            q = np.rad2deg(q)
        plt.figure(1)
        plt.plot(t, q, label=f'q{i+1}')



    for i, qd in enumerate([qd1, qd2, qd3, qd4, qd5, qd6]):
        if in_degrees:
            qd = np.rad2deg(qd)
        plt.figure(2)
        plt.plot(t, qd, label=f'qd{i+1}')

    plt.show()

    """
    for i, tcp_coords in zip(['x', 'y', 'z'], [x, y, z]):

        # axs[2].plot(t, tcp_coords, label=f'{i}')

    for i, tcp_orient in zip(['rx', 'ry', 'rz'], [rx, ry, rz]):
        if in_degrees:
            tcp_orient = np.rad2deg(tcp_orient)

        axs[3].plot(t, tcp_orient, label=f'{i}')


    for i, speed in enumerate([sp1, sp2, sp3, sp4, sp5, sp6]):
            axs[4].plot(t, speed, label=f'vel tcp{i+1}')

    # plot accelerations

    for i, speed in enumerate([sp1, sp2, sp3, sp4, sp5, sp6]):
        print('Zeitticks: ', len(t))


    for i, qt in enumerate([qt1, qt2, qt3, qt4, qt5, qt6]):
        if in_degrees:
            qt = np.rad2deg(qt)
        axs[5].plot(t, qt, label=f'qt{i+1}')


    for i, qdt in enumerate([qdt1, qdt2, qdt3, qdt4, qdt5, qdt6]):
        if in_degrees:
            qdt = np.rad2deg(qdt)
            axs[6].plot(t, qdt, label=f'qdt{i+1}')


    for i, qddt in enumerate([qddt1, qddt2, qddt3, qddt4, qddt5, qddt6]):
        if in_degrees:
            qddt = np.rad2deg(qddt)
            axs[7].plot(t, qddt, label=f'qddt{i+1}')

    
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    axs[3].legend()
    axs[4].legend()
    axs[5].legend()
    axs[6].legend()
    axs[7].legend()

    axs[0].grid()
    axs[1].grid()
    axs[2].grid()
    axs[3].grid()
    axs[4].grid()
    axs[5].grid()
    axs[6].grid()
    axs[7].grid()
    

    plt.locator_params(axis='x', nbins=50)
    # axs[2].locator_params(axis='y', nbins=10)
    plt.xticks(rotation=25)
    plt.xlabel('System Time')

    plt.legend()
    plt.show()

    """



def calc_acceleration(v_diff, t_diff):
    # a = dv/dt
    a = v_diff/t_diff
    print(a)
    return a


def plot_time_diff(filename):
    with open(filename, 'r') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)

        t = []

        for row in reader:
            t.append(row[0])

        diff = np.diff(np.array(t))
        print(diff)
        plt.plot(t[:-1], diff)
        plt.show()
        print(diff)

def log_data(filename):
    # t = 7704.35
    # qn = np.array([-0.25869615, 1.22177154, -2.0944236, 0.0038536, 2.23060868, 1.85355]).tolist()
    # pose = np.array([-0.25869615, -0.30473564, 0.74292051, 0.0038536, 2.23060868, -1.45804981]).tolist()
    # speed = np.array([0, 0, 0, 0, 0, 0]).tolist()

    with open(filename, 'w', newline='') as f:
        t = 7704.35
        qn = np.array([-0.25869615, 1.22177154, -2.0944236, 0.0038536, 2.23060868, 1.85355]).tolist()
        pose = np.array([-0.25869615, -0.30473564, 0.74292051, 0.0038536, 2.23060868, -1.45804981]).tolist()
        speed = np.array([0, 0, 0, 0, 0, 0]).tolist()

        writer = csv.writer(f)
        # writer.writerow(['Time', 'Joint Angles', 'Pose', 'Speed'])
        writer.writerow([t, *qn, *pose, *speed])

        t = 7804.35
        qn = np.array([-0.234980, 2.2340985,  -2.2144236, 0.2438536, 2.6567860868, 0.85355]).tolist()
        pose = np.array([-0.124269615, -1.30473564, 2.74292051, 0.12338536, 2.00860868, -1.5704981]).tolist()
        speed = np.array([0, 0, 0, 0, 0, 0]).tolist()
        writer.writerow([t, *qn, *pose, *speed])


def convert_old_csv(infile, outfile):

    # first open output file to write to
    with open(outfile, 'w', newline='') as outf:
        writer = csv.writer(outf, delimiter=',')

        with open(infile, 'r', newline='') as inf:
            reader = csv.reader(inf, delimiter=',')

            for row in reader:
                # row = reader.__next__()
                new_arr = []
                row = str(row)
                row = row.replace('[', '')
                row = row.replace(']', '')
                row = row.replace(',', ' ')
                row = row.replace('\'', ' ')
                # row = ','.join(row.split())

                writer.writerow(row.split())


def plot_one(filename, row_num, in_degrees=False):
    with open(filename, 'r') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
        time = []
        data = []

        for row in reader:
            time.append(row[0])
            data.append(row[row_num])
            # print('Time:' ,time)
            # print('Data:' ,data)

        plt.figure(1)
        if in_degrees:
            plt.plot(time, np.rad2deg(data))
        else:
            plt.plot(time, data)
        plt.grid()
        plt.show()

def plot(filename, range, in_degrees=False):
    data = pd.read_csv(filename, header=None)
    print(data, end='\n\n')

def add_headers(filename):
    names = ['t', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6', 'x', 'y', 'z', 'rx',
             'ry', 'rz', 'sp1', 'sp2', 'sp3', 'sp4', 'sp5', 'sp6', 'qt1', 'qt2', 'qt3', 'qt4', 'qt5', 'qt6',
             'qdt1', 'qdt2', 'qdt3', 'qdt4','qdt5', 'qdt6',
             'qddt1', 'qddt2', 'qddt3', 'qddt4', 'qddt5', 'qddt6']

    data = pd.read_csv(filename, sep=',', header=None)

    if data[0][0] == 't':
        print('File already has headers.')
        return

    data.columns = names
    new_f_name = filename[:-4]
    data.to_csv(new_f_name + '_named.csv', sep=',', index=False)

def plot_columns(filename, columns):
    names = ['t', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6', 'x', 'y', 'z', 'rx',
             'ry', 'rz', 'sp1', 'sp2', 'sp3', 'sp4', 'sp5', 'sp6', 'qt1', 'qt2', 'qt3', 'qt4', 'qt5', 'qt6',
             'qdt1', 'qdt2', 'qdt3', 'qdt4','qdt5', 'qdt6',
             'qddt1', 'qddt2', 'qddt3', 'qddt4', 'qddt5', 'qddt6']
    data = pd.read_csv(filename, sep=',', header=None)

    data.columns = names

    print(data)
    data.plot(x='t', y=columns)
    plt.show()

if __name__ == '__main__':
    # log_data('conv_test.csv')
    # convert_old_csv('raw_logs/einzelachs_movej_ohne_t2.csv', 'conv_logs/einzelachs_movej_ohne_t2.csv')
    # plot_data('conv_logs/conv_test.csv')
    #plot_time_diff('test.csv')

    # plot_data('../logs/Daniel/4_linear_deltaRZ_new_11_19_35.csv', in_degrees=True, selector=('q', 'qd', 'pose', 'tcp_v', 'qt', 'qdt', 'qddt'))
    # plot_data_single_windows('../logs/Daniel/4_linear_deltaRZ_new_11_19_35.csv', in_degrees=True)
    # plot_one('logs/4_linear_deltaRZ_new_11_19_35.csv', row_num=3, in_degrees=True)
    #plot('logs/1_elbow_30_13_07_21.csv', range=2)
    # add_headers('logs/2_elbow_30, t_4_13_08_28.csv')

    # plot = Plot()
    # df = plot.to_pd('logs/1_elbow_30_13_07_21_named.csv', has_header=0)
    # plot.plot_df(df, columns=['q1', 'q2', 'q3'])

    plot_columns('logs/1_elbow_30_13_07_21.csv', columns=['q1', 'q2', 'q3', 'q4'])
