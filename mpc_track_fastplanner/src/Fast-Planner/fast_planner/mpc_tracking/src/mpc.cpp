#include "mpc_tracking/mpc.h"
#include <cppad/ipopt/solve.hpp>
#include <ros/ros.h>
using CppAD::AD;

const int N = 30;
const double dt = 0.1;

int x_start = 0;
int y_start = x_start + N;
int psi_start = y_start + N;
int u_start = psi_start + N;
int v_start = u_start + N - 1;
int r_start = v_start + N - 1;
double num = 0;

Mpc::Mpc() {}
Mpc::~Mpc() {}
class FG_eval
{
public:
    Eigen::MatrixXd desired_state_;
    double car_obstacle_dist;
    Eigen::Vector2d gradient_; // 新增梯度方向成员变量
    FG_eval(Eigen::MatrixXd desired_state,double dist,Eigen::Vector2d gradient) {
        desired_state_ = desired_state;
        car_obstacle_dist = dist;
        gradient_ = gradient; // 初始化梯度方向
    }
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        //cost
        fg[0] = 0;

        //weights
        const int x_weight = 10;
        const int y_weight = 10;
        const int psi_weight =0;
        const int u_weight = 1;
        const int v_weight = 1;

        for (int i = 0; i < N; ++i) {
            AD<double> x_des = desired_state_(i, 0);
            AD<double> y_des = desired_state_(i, 1);
            AD<double> psi_des = desired_state_(i, 2);
            fg[0] += x_weight * CppAD::pow((vars[x_start + i] - x_des), 2);
            fg[0] += y_weight * CppAD::pow((vars[y_start + i] - y_des), 2);
            if (psi_des > M_PI) psi_des -= 2 * M_PI;
            if (psi_des < -M_PI) psi_des += 2 * M_PI;
            fg[0] += psi_weight * CppAD::pow(vars[psi_start + i] - psi_des + M_PI / 2, 2);
        }
        for (int i = 0; i < N-1; ++i) {
            fg[0] += u_weight * CppAD::pow(vars[u_start + i ] , 2);
            fg[0] += v_weight * CppAD::pow(vars[v_start + i ] , 2);
        }
        // 新增避障速度方向约束
    for (int i = 0; i < N-1; ++i) {
        AD<double> u_i = vars[u_start + i];
        AD<double> v_i = vars[v_start + i];
        AD<double> projection = (u_i * gradient_.x() + v_i * gradient_.y());//*car_obstacle_dist;
        fg[1 + 3*N + i] = projection; // 约束为投影>=0
    }
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        
        for (int i = 1; i < N; ++i) {
            AD<double> x_1 = vars[x_start + i];
            AD<double> y_1 = vars[y_start + i];
            AD<double> psi_1 = vars[psi_start + i];

            AD<double> x_0 = vars[x_start + i - 1];
            AD<double> y_0 = vars[y_start + i - 1];
            AD<double> psi_0 = vars[psi_start + i - 1];

            AD<double> u_0 = vars[u_start + i - 1];
            AD<double> v_0 = vars[v_start + i - 1];
            AD<double> r_0 = vars[r_start + i - 1];

            fg[1 + x_start + i] = x_1 - (x_0 + (u_0 * CppAD::cos(psi_0) - v_0 * CppAD::sin(psi_0)) * dt);
            fg[1 + y_start + i] = y_1 - (y_0 + (u_0 * CppAD::sin(psi_0) + v_0 * CppAD::cos(psi_0)) * dt);
            fg[1 + psi_start + i] = psi_1 - (psi_0 + r_0 * dt);
        }
    }
};

vector<double> Mpc::solve(Eigen::Vector3d state, Eigen::MatrixXd desired_state,double dist,Eigen::Vector2d gradient,double safe_distance) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state(0);
    double y = state(1);
    double psi = state(2);

    int n_var = N * 3 + (N - 1) * 3;
    int n_constraints = N * 3 + (N - 1); // 原3N个等式约束 + (N-1)个不等式约束
    Dvector vars(n_var);
    for (int i = 0; i < n_var; ++i) {
        vars[i] = 0.0;
    }

    Dvector vars_lowerbound(n_var);
    Dvector vars_upperbound(n_var);

    for (int i = 0; i < u_start; ++i) {
        vars_lowerbound[i] = -1e19;
        vars_upperbound[i] = 1e19;
    }
    for (int i = u_start; i < v_start; ++i) {
        vars_lowerbound[i] = -1.5;
        vars_upperbound[i] = 1.5;
    }
    for (int i = v_start; i < r_start; ++i) {
        vars_lowerbound[i] = -1.5;
        vars_upperbound[i] = 1.5;        
    }
    for (int i = r_start; i < n_var; ++i) {
        vars_lowerbound[i] = -1;
        vars_upperbound[i] = 1;
    }

Dvector constraints_lowerbound(n_constraints);
Dvector constraints_upperbound(n_constraints);

// 原等式约束上下界设为0
for (int i = 0; i < 3*N; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
}

// 新增不等式约束的上下界
for (int i = 3*N; i < n_constraints; ++i) {
    // 当距离小于安全距离时施加约束，否则放宽约束
    if (dist < safe_distance) {
        constraints_lowerbound[i] = 0.0; // 速度投影必须非负
        constraints_upperbound[i] = 1e19;
        ROS_WARN("HEAD!!!");
    } else {
        constraints_lowerbound[i] = -1e19; // 无约束
        constraints_upperbound[i] = 1e19;
    }
}

// 初始状态约束保持不变
constraints_lowerbound[x_start] = x;
constraints_lowerbound[y_start] = y;
constraints_lowerbound[psi_start] = psi;
constraints_upperbound[x_start] = x;
constraints_upperbound[y_start] = y;
constraints_upperbound[psi_start] = psi;

    FG_eval fg_eval(desired_state,dist,gradient);

    string options;
    options += "Integer print_level 0\n";
    options += "Sparse true forward\n";
    options += "Sparse true reverse\n";
    options += "Numeric max_cpu_time 0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FG_eval> (
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution
    );

    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    auto cost = solution.obj_value;
    //cout << "cost:" << cost << " ";

    vector<double> solved;
    solved.push_back(solution.x[u_start]);
    solved.push_back(solution.x[v_start]);
    for (int i = 0; i < N; ++i) {
        solved.push_back(solution.x[x_start + i]);
        solved.push_back(solution.x[y_start + i]);
    }
    cout << "u:" << solution.x[u_start] << " " << "v:" << solution.x[v_start] << endl;
    return solved;
}