#ifndef EKF_HPP
#define EKF_HPP

#include <eigen3/Eigen/Core>
#include <iostream>

//卡尔曼滤波器
//状态量    观测量

// 卡尔曼 State_NUM：状态量的数量 Measure_NUM:观测量的数量 规定几*几 
template<int State_NUM, int Measure_NUM>
class Ekf {
public:
    //状态转移矩阵
    typedef Eigen::Matrix<double, State_NUM, State_NUM> F;
    //测量矩阵
    typedef Eigen::Matrix<double, Measure_NUM, State_NUM> H;
    //状态的雅可比矩阵
    typedef Eigen::Matrix<double, State_NUM, State_NUM> FJacobi;
    //测量的雅可比矩阵
    typedef Eigen::Matrix<double, Measure_NUM, State_NUM> HJacobi;
    //过程噪声协方差矩阵
    typedef Eigen::Matrix<double, State_NUM, State_NUM> Q;
    //测量噪声协方差矩阵
    typedef Eigen::Matrix<double, Measure_NUM, Measure_NUM> R;
    //状态协方差矩阵
    typedef Eigen::Matrix<double, State_NUM, State_NUM> P;
    //状态向量
    typedef Eigen::Matrix<double, State_NUM, 1> State;
    //测量向量
    typedef Eigen::Matrix<double, Measure_NUM, 1> Measurement;
	//构造函数 初始化Ekf对象
    Ekf(F f, H h, FJacobi f_j, HJacobi h_j, Q q, R r, P p):
        m_f(f),
        m_h(h),
        m_f_jacobi(f_j),
        m_h_jacobi(h_j),
        m_q(q),
        m_r(r),
        m_p(p){
            std::cout << "init successfully" << std::endl;
    }
    //设置滤波的初始状态向量
    void initState(const State& init_state) {
        m_state = init_state;
        m_pre_state = init_state;
    }
    //卡尔曼滤波的五个重要公式
    void predict() {
        // 1.预测下一个状态
        m_pre_state = m_f * m_state;

        // 2.预测状态协方差矩阵
        m_pre_p = m_f * m_p * m_f.transpose() + m_q;
    }

    Measurement h(const State& state) {
        return m_h * state;  // 使用测量矩阵 m_h 将状态转换为测量
    }

    //更新步骤
    void update(Measurement measurement) {
        // 3.计算卡尔曼增益
        m_k =  m_pre_p * m_h.transpose() * ((m_h * m_pre_p * m_h.transpose() + m_r).inverse());
        //4.更新状态
        m_state = m_pre_state + m_k * (measurement - h(m_pre_state));
        // 5.更新协方差
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_p.rows(), m_p.cols());
        m_p = (I - m_k * m_h) * m_pre_p;
    }
    //提供访问当前状态向量和协方差矩阵的接口
    State getState() {return m_state;}
    P getP() {return m_p;}
    //虚析构函数
    virtual ~Ekf() = default;
private:
    F m_f;              //转移状态矩阵
    H m_h;              //测量矩阵
    FJacobi m_f_jacobi; //状态雅可比矩阵
    HJacobi m_h_jacobi; //测量雅可比矩阵
    Q m_q;              //过程噪声协方差矩阵
    R m_r;              //测量噪声协方差矩阵
    P m_p;              //当前状态协方差矩阵
    P m_pre_p;          //预测状态协方差矩阵
    State m_state;      //当前状态向量
    State m_pre_state;  //预测状态向量
    //动态大小矩阵类型 存储卡尔曼增益矩阵
    Eigen::MatrixXd m_k;
    Measurement m_measurement;
};

#endif //EKF_HPP