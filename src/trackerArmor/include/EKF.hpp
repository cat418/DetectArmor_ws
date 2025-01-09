#include <eigen3/Eigen/Core>
#include <iostream>

//卡尔曼滤波器
//状态量    观测量
template<int State_NUM, int Measure_NUM>
class Ekf {
public:
    typedef Eigen::Matrix<double, State_NUM, State_NUM> F;//状态转移矩阵
    typedef Eigen::Matrix<double, Measure_NUM, Measure_NUM> H;//观测矩阵
    typedef Eigen::Matrix<double, State_NUM, State_NUM> FJacobi;//状态转移矩阵的雅可比矩阵
    typedef Eigen::Matrix<double, Measure_NUM, State_NUM> HJacobi;//观测矩阵的雅可比矩阵
    typedef Eigen::Matrix<double, State_NUM, State_NUM> Q;//系统过程噪声的协方差矩阵
    typedef Eigen::Matrix<double, Measure_NUM,Measure_NUM> R;//测量噪声的协方差矩阵
    typedef Eigen::Matrix<double, State_NUM, State_NUM> P;//状态协方差矩阵
    typedef Eigen::Matrix<double, 1, State_NUM> State;//表示一个状态向量
    typedef Eigen::Matrix<double, 1, Measure_NUM> Measurement;//表示一个测量向量

    //构造函数
    Ekf( F f, H h, FJacobi f_j, HJacobi h_j, Q q, R r, P p ):
        m_f( f ),
        m_h( h ),
        m_f_jacobi( f_j ),
        m_h_jacobi( h_j ),
        m_q( q ),
        m_r( r ),
        m_p( p ){
            std::cout << "init successfully" << std::endl;
    }

    //设置初始状态
    void setInitialState( const State& state ) {
        m_state = state;
    }

    //预测步骤
    void predict() {
        m_pre_state = m_f *m_state;//预测状态
        m_pre_p = m_f *m_p * m_f.transpose() + m_q;//预测协方差矩阵
    }

    //更新步骤
    void update(Measurement measure) {
        Eigen::Matrix<double, Measure_NUM, Measure_NUM> s = m_h * m_pre_p * m_h.transpose() + m_r;//创新协方差
        m_k = m_pre_p *m_h.transpose() * s.inverse();//卡尔曼增益
        m_state = m_pre_state + m_k * (Measurement - m_h * m_pre_state);//更新状态
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity( m_p.rows(), m_p.cols());//单位矩阵
        m_p = (I - m_k * m_h ) * m_pre_p;// 更新协方差矩阵
    }
    //获取当前状态
    State getState() const { return m_state; }
    //获取当前协方差矩阵
    P getP() { return m_p; }
    //析构函数
    virtual ~Ekf() {};
private:
    F m_f;//状态转移矩阵
    H m_h;//观测矩阵
    FJacobi m_f_jacobi;//状态转移矩阵的雅可比矩阵
    HJacobi m_h_jacobi;//观测矩阵的雅可比矩阵
    Q m_q;//系统过程噪声协方差矩阵
    R m_r;//观测噪声协方差矩阵
    P m_p;//状态协方差矩阵
    P m_pre_p;//预测协方差矩阵
    State m_state;//当前状态
    State m_pre_state;//预测状态
    Eigen::MatrixXd m_k;//卡尔曼增益
    Measurement m_measure;//保存当前测量值的变量
};