import csv
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


class Plot():
    def __init__(self):
        self.t = ['t']
        self.q = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
        self.qd = ['qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6']
        self.pose = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        self.speed = ['sp1', 'sp2', 'sp3', 'sp4', 'sp5', 'sp6']
        self.qt = ['qt1', 'qt2', 'qt3', 'qt4', 'qt5', 'qt6']
        self.qdt = ['qdt1', 'qdt2', 'qdt3', 'qdt4', 'qdt5', 'qdt6']
        self.qddt = ['qddt1', 'qddt2', 'qddt3', 'qddt4', 'qddt5', 'qddt6']

    def to_pd(self, filename, has_header=False):
        data = pd.read_csv(filename, sep=',', header=has_header)
        return data

    def add_headers(self, filename, write=False):
        names = ['t', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
                 'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6',
                 'x', 'y', 'z', 'rx','ry', 'rz',
                 'sp1', 'sp2', 'sp3', 'sp4', 'sp5', 'sp6',
                 'qt1', 'qt2', 'qt3', 'qt4', 'qt5', 'qt6',
                 'qdt1', 'qdt2', 'qdt3', 'qdt4', 'qdt5', 'qdt6',
                 'qddt1', 'qddt2', 'qddt3', 'qddt4', 'qddt5', 'qddt6']

        # should also work
        names = self.t + self.q + self.qd + self.pose + self.speed+ self.qt + self.qdt + self.qddt
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

    def plot_columns(self, filename, columns, y_label_l=None, y_label_r=None, title=None, show_degrees=False):
        data = pd.read_csv(filename, sep=',', header=0)


        ax = data.plot(x='t', y=columns, title=title)

        plt.grid(which='major', color='#666666', linestyle='-')
        plt.minorticks_on()
        plt.grid(which='minor', color='#999999', linestyle='-', alpha=0.2)

        ax.set_xlabel('t in s')
        ax.set_ylabel(y_label_l)

        if show_degrees:
            ax2 = ax.twinx()
            ax2.set_ylabel(y_label_r)
            ax2.set_zorder(0)

            self.convert_ax2_to_degrees(ax, ax2)

    def plot_all(self, filename, show_degrees=False):
        data = pd.read_csv(filename, sep=',', header=0)

        columns = (self.q, self.qd, self.pose, self.speed, self.qt, self.qdt, self.qddt)


        for c in columns:
            ax = data.plot(x='t', y=c, title=filename)
            ax.set_xlabel('t in s')
            plt.grid(which='major', color='#666666', linestyle='-')
            plt.minorticks_on()
            plt.grid(which='minor', color='#999999', linestyle='-', alpha=0.2)
            if show_degrees:

                ax2 = ax.twinx()
                ax2.set_ylabel('Degrees')
                ax2.set_zorder(0)

                self.convert_ax2_to_degrees(ax, ax2)

    def convert_ax2_to_degrees(self, ax, ax2):
        """
        Update second axis according with first axis.
        """
        y1, y2 = ax.get_ylim()
        # print(y1, y2)
        ax2.set_ylim(np.rad2deg(y1), np.rad2deg(y2))
        ax2.figure.canvas.draw()

    def rad_2_deg(self):
        pass

    def deg_2_rad(self):
        pass

    def reset_time(self, filename):
        data = pd.read_csv(filename, sep=',', header=0)

        start_time = data['t'][0]

        def sub_start(t):
            return t-start_time

        data['t'] = data['t'].apply(sub_start)
        new_f_name = filename[:-4]
        data.to_csv(new_f_name + '_zeroed.csv', sep=',', index=False)


    def set_ticks(self, x=None, y=None):
        """Doesnt work"""
        if x:
            plt.locator_params(axis='x', nbins=x, tight=True)
        if y:
            plt.locator_params(axis='y', nbins=y, tight=True)


if __name__ == '__main__':
    plot = Plot()

    # post-processing
    plot.add_headers('2_elbow_30, t_4_11_27_51.csv', write=False)
    plot.reset_time('2_elbow_30, t_4_11_27_51_named.csv')

    # plotting
    # plot.plot_columns('logs/4_linear_deltaRZ_new_11_19_35_named_zeroed.csv', columns=plot.q,
    #                  title='Motorwinkel', y_label_l='Rad', y_label_r='Grad')

    # plot.plot_columns('logs/1_elbow_30_13_07_21_named_zeroed.csv', columns=plot.q,
    #                    title='Motorwinkel', y_label_l='Rad', y_label_r='Grad')

    # plot.plot_columns('logs/1_elbow_30_13_07_21_named_zeroed.csv', columns=plot.q, show_degrees=True,
    #                    title='Motorwinkel', y_label_l='Rad', y_label_r='Grad')
    # plot.plot_columns('logs/3_linear_deltaZ_new_11_11_17_named_zeroed.csv', columns=plot.pose,
    #                   title='Motorwinkel', y_label_l='Rad', y_label_r='Grad')
    # plot.plot_columns('logs/2_elbow_30, t_4_13_10_53_named_zeroed.csv', columns=plot.q,
    #                   title='Motorwinkel', y_label_l='Rad', y_label_r='Grad')

    #plot.plot_all('2_elbow_30, t_4_11_27_51_named_zeroed.csv', show_degrees=True)
    # plot.plot_all('logs/2_elbow_30, t_4_13_08_28_named_zeroed.csv', show_degrees=True)
    # plot.plot_all('logs/3_linear_deltaZ_new_11_11_17_named_zeroed.csv', show_degrees=True)
    # plot.plot_all('logs/4_linear_deltaRZ_new_11_19_35_named_zeroed.csv', show_degrees=True)

    plt.show()
