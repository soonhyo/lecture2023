import argparse
import math
import matplotlib.pyplot as plt
from casadi import *

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion


# 以下を参考に
# https://gist.github.com/mayataka/7e608dbbbcd93d232cf44e6cab6b0332

class MobileRobot:
    def __init__(self):
        self.r = 0.2   # 車輪半径[m]
        self.d = 0.32    # 車輪間距離[m]
        self.ga = 9.81   # 重力加速度[m/s2]

    def dynamics(self, x, u):
        r = self.r
        d = self.d
        mx = x[0]    # mobile robotのx方向位置[m]
        my = x[1]    # mobile robotのy方向位置[rad]
        mth = x[2]   # mobile robotの回転方向theta[rad]
        wvr = u[0]   # 右車輪の速度[rad/s]
        wvl = u[1]   # 左車輪の速度[rad/s]
        # cart の水平加速度
        vr = r*wvr
        vl = r*wvl
        v = (vr+vl)/2
        w = (vr-vl)/(2*d)
        dx = v*cos(mth)
        dy = v*sin(mth)
        dth = w
        return dx, dy, dth

# コスト関数を記述
class CostFunction:
    def __init__(self, x_ref):
        self.nx = 3
        self.nu = 2
        self.x_ref = x_ref    # 目標状態
        # ステージコストのパラメータ
        self.Q  = [1.0, 1.0, 0.0]       # 状態への重み
        self.R  = [0.1, 0.1]            # 制御入力への重み
        # 終端コストのパラメータ
        self.Qf = [1.0, 1.0, 0.0]       # 状態への重み

    # ステージコスト
    def stage_cost(self, x, u):
        l = 0
        for i in range(self.nx):
            l += 0.5 * self.Q[i] * (x[i]-self.x_ref[i])**2
        for i in range(self.nu):
            l += 0.5 * self.R[i] * u[i]**2
        return l

    # 終端コスト
    def terminal_cost(self, x):
        Vf = 0
        for i in range(self.nx):
            Vf += 0.5 * self.Qf[i] * (x[i]-self.x_ref[i])**2
        return Vf


class MPC:
    def __init__(self, x_ref):
        # 問題設定
        T = 2.0     # ホライゾン長さ (MPCなので短め)
        N = 50      # ホライゾン離散化グリッド数 (MPCなので荒め)
        dt = T / N  # 離散化ステップ
        nx = 3      # 状態空間の次元
        nu = 2      # 制御入力の次元
        mobilerobot = MobileRobot() # mobilerobotのダイナミクス
        cost_function = CostFunction(x_ref) # コスト関数

        # 以下で非線形計画問題(NLP)を定式化
        w = []    # 最適化変数を格納する list
        w0 = []   # 最適化変数(w)の初期推定解を格納する list
        lbw = []  # 最適化変数(w)の lower bound を格納する list
        ubw = []  # 最適化変数(w)の upper bound を格納する list
        J = 0     # コスト関数 
        g = []    # 制約（等式制約，不等式制約どちらも）を格納する list
        lbg = []  # 制約関数(g)の lower bound を格納する list
        ubg = []  # 制約関数(g)の upper bound を格納する list
        lam_x0 = []  # 制約 lbw<w<ubw のラグランジュ乗数
        lam_g0 = []  # 制約 lbg<g<ubg のラグランジュ乗数

        Xk = MX.sym('X0', nx) # 初期時刻の状態ベクトル x0
        w += [Xk]             # x0 を 最適化変数 list (w) に追加
        # 初期状態は given という条件を等式制約として考慮
        lbw += [0.0, 0.0, 0.0]      # 等式制約は lower-bound と upper-bound を同じ値にすることで設定
        ubw += [0.0, 0.0, 0.0]      # 等式制約は lower-bound と upper-bound を同じ値にすることで設定
        w0 +=  [0.0, 0.0, 0.0]      # x0 の初期推定解
        lam_x0 += [0.0, 0.0, 0.0]   # ラグランジュ乗数の初期推定解

        # 離散化ステージ 0~N-1 までのコストと制約を設定
        for k in range(N):
            Uk = MX.sym('U_' + str(k), nu) # 時間ステージ k の制御入力 uk を表す変数
            w   += [Uk]                    # uk を最適化変数 list に追加
            lbw += [-5.0, -5.0]            # uk の lower-bound
            ubw += [5.0, 5.0]              # uk の upper-bound
            w0  += [0.0, 0.0]              # uk の初期推定解
            lam_x0 += [0.0, 0.0]           # ラグランジュ乗数の初期推定解

            # ステージコスト
            J = J + dt * cost_function.stage_cost(Xk, Uk) # コスト関数にステージコストを追加

            # Forward Euler による離散化状態方程式
            dXk = mobilerobot.dynamics(Xk, Uk)
            Xk_next = vertcat(Xk[0] + dXk[0] * dt,
                              Xk[1] + dXk[1] * dt,
                              Xk[2] + dXk[2] * dt)
            Xk1 = MX.sym('X_' + str(k+1), nx)  # 時間ステージ k+1 の状態 xk+1 を表す変数
            w   += [Xk1]                       # xk+1 を最適化変数 list に追加
            lbw += [-inf, -inf, -inf]          # xk+1 の lower-bound （指定しない要素は -inf）
            ubw += [inf, inf, inf]             # xk+1 の upper-bound （指定しない要素は inf）
            w0 += [0.0, 0.0, 0.0]              # xk+1 の初期推定解
            lam_x0 += [0, 0, 0]                # ラグランジュ乗数の初期推定解

            # 状態方程式(xk+1=xk+fk*dt) を等式制約として導入
            g   += [Xk_next-Xk1]
            lbg += [0.0, 0.0, 0.0]     # 等式制約は lower-bound と upper-bound を同じ値にすることで設定
            ubg += [0.0, 0.0, 0.0]     # 等式制約は lower-bound と upper-bound を同じ値にすることで設定
            lam_g0 += [0.0, 0.0, 0.0]   # ラグランジュ乗数の初期推定解
            Xk = Xk1

        # 終端コスト 
        J = J + cost_function.terminal_cost(Xk) # コスト関数に終端コストを追加

        self.J = J
        self.w = vertcat(*w)
        self.g = vertcat(*g)
        self.x = w0
        self.lam_x = lam_x0
        self.lam_g = lam_g0
        self.lbx = lbw
        self.ubx = ubw
        self.lbg = lbg
        self.ubg = ubg

        # 非線形計画問題(NLP)
        self.nlp = {'f': self.J, 'x': self.w, 'g': self.g} 
        # Ipopt ソルバー，最小バリアパラメータを0.1，最大反復回数を5, ウォームスタートをONに
        self.solver = nlpsol('solver', 'ipopt', self.nlp, {'calc_lam_p':True, 'calc_lam_x':True, 'print_time':False, 'ipopt':{'max_iter':5, 'mu_min':0.1, 'warm_start_init_point':'yes', 'print_level':0, 'print_timing_statistics':'no'}}) 
        # self.solver = nlpsol('solver', 'scpgen', self.nlp, {'calc_lam_p':True, 'calc_lam_x':True, 'qpsol':'qpoases', 'print_time':False, 'print_header':False, 'max_iter':5, 'hessian_approximation':'gauss-newton', 'qpsol_options':{'print_out':False, 'printLevel':'none'}}) # print をオフにしたいがやり方がわからない

    def init(self, x0=None):
        if x0 is not None:
            # 初期状態についての制約を設定
            nx = x0.shape[0]
            self.lbx[0:nx] = x0
            self.ubx[0:nx] = x0
        # primal variables (x) と dual variables（ラグランジュ乗数）の初期推定解も与えつつ solve（warm start）
        sol = self.solver(x0=self.x, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg, lam_x0=self.lam_x, lam_g0=self.lam_g)
        # 次の warm start のために解を保存
        self.x = sol['x'].full().flatten()
        self.lam_x = sol['lam_x'].full().flatten()
        self.lam_g = sol['lam_g'].full().flatten()

    def solve(self, x0):
        # 初期状態についての制約を設定
        nx = x0.shape[0]
        self.lbx[0:nx] = x0
        self.ubx[0:nx] = x0
        # primal variables (x) と dual variables（ラグランジュ乗数）の初期推定解も与えつつ solve（warm start）
        sol = self.solver(x0=self.x, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg, lam_x0=self.lam_x, lam_g0=self.lam_g)
        # 次の warm start のために解を保存
        self.x = sol['x'].full().flatten()
        self.lam_x = sol['lam_x'].full().flatten()
        self.lam_g = sol['lam_g'].full().flatten()
        return np.array(self.x[3:5]) # 制御入力を return


class MujocoMPC:
    def __init__(self, x_ref):
        rospy.init_node("mobile_mpc_node")
        ctrl_ref_pub = rospy.Publisher("mujoco_ctrl_ref", Float32MultiArray, queue_size=1)
        state_sub = rospy.Subscriber("joint_mujoco_states", JointState, self.state_callback, queue_size=1)
        rospy.sleep(rospy.Duration(0.5))

        sim_time = 10.0 # 10秒間のシミュレーション
        sampling_time = 0.01 # 0.01秒（10ms）のサンプリング周期
        sim_steps = math.floor(sim_time/sampling_time)
        xs = []
        us = []
        mobilerobot = MobileRobot()
        mpc = MPC(x_ref)
        mpc.init()
        x = np.zeros(3)

        self.current_pos = None
        rate = rospy.Rate(1.0/sampling_time)
        for step in range(sim_steps):
            if step%(1/sampling_time)==0:
                print('t =', step*sampling_time)
            u = mpc.solve(x)

            ctrl_ref_msg = Float32MultiArray()
            ctrl_ref_msg.data = np.concatenate([u, [0.0]])
            ctrl_ref_pub.publish(ctrl_ref_msg)

            xs.append(x)
            us.append(u)
            if self.current_pos is not None:
                x = np.array(self.current_pos)
            else:
                x1 = x + sampling_time * np.array(mobilerobot.dynamics(x, u))
                x = x1
            rate.sleep()
            if rospy.is_shutdown():
                break
        ctrl_ref_msg = Float32MultiArray()
        ctrl_ref_msg.data = [0.0, 0.0, 0.0]
        ctrl_ref_pub.publish(ctrl_ref_msg)

        # シミュレーション結果をプロット
        xs1 = [x[0] for x in xs]
        xs2 = [x[1] for x in xs]
        xs3 = [x[2] for x in xs]
        us1 = [u[0] for u in us]
        us2 = [u[1] for u in us]
        tgrid = [sampling_time*k for k in range(sim_steps)]

        plt.figure(1)
        plt.clf()
        plt.plot(tgrid, xs1, '--')
        plt.plot(tgrid, xs2, '-')
        plt.plot(tgrid, xs3, '-')
        plt.step(tgrid, us1, '-.')
        plt.step(tgrid, us2, '-.')
        plt.xlabel('t')
        plt.legend(['x(x1)','y(x2)', 'th(x3)', 'wvr(u1)','wvl(u2)'])
        plt.grid()
        plt.show()

    def state_callback(self, msg):
        xyzquat = [msg.position[i] for i, name in enumerate(msg.name) if "mobile" in name]
        xyz = xyzquat[0:3]
        q = xyzquat[3:]
        rpy = euler_from_quaternion([q[1], q[2], q[3], q[0]])
        self.current_pos = [xyz[0], xyz[1], rpy[2]]
        print(self.current_pos)

def main():
    parser = argparse.ArgumentParser(
        description="mobile mpc")
    parser.add_argument("--x_ref", '-x', type=float, nargs="+", required=True,
                        help='[x, y, th]')
    args = parser.parse_args()
    print(args.x_ref)
    args = parser.parse_args()
    mujocoMPC = MujocoMPC(args.x_ref)

if __name__ == '__main__':
    main()
