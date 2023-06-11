#include <iostream>

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <opencv2/core/core.hpp>

using namespace std;

class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置
    virtual void setToOriginImpl() override { _estimate << 0, 0, 0; }

    // 更新
    virtual void oplusImpl(const double *update) override {
        _estimate += Eigen::Vector3d(update);
    }

    // 存盘和读盘：留空
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    // 计算曲线模型误差
    virtual void computeError() override {
        const CurveFittingVertex *v =
            static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x +
                                               abc(1, 0) * _x + abc(2, 0));
    }

    // 计算雅可比矩阵
    virtual void linearizeOplus() override {
        const CurveFittingVertex *v =
            static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

  public:
    double _x; // x 值， y 值为 _measurement
};

int main(int agrc, char **argv) {
    double ar = 1.0, br = 2.0, cr = 1.0; // 真实参数值
    double ae = 0, be = 0, ce = 0;       // 估计参数值

    int N = 100;
    double w_sigma = 1.0; // Noise
    double inv_sigma = 1.0 / w_sigma;

    cv::RNG rng;

    // 构造数据
    std::vector<double> x_data, y_data;
    for (int i = 0; i < N; ++i) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) +
                         rng.gaussian(w_sigma * w_sigma));
    }

    // std::unique_ptr<BlockSolverType::LinearSolverType> linear_solver(
    //     new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>());
    // std::unique_ptr<BlockSolverType> solver_ptr(
    //     new BlockSolverType(std::move(linear_solver)));
    // g2o::OptimizationAlgorithmLevenberg *solver =
    //     new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    // g2o::SparseOptimizer optimizer;
    // optimizer.setAlgorithm(solver);

    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>
        Block; // 每个误差项优化变量维度为3，误差值维度为1

    // step 1: 创建一个线性求解器LinearSolver
    // Block::LinearSolverType *linearsolver =
    //     new g2o::LinearSolverDense<Block::PoseMatrixType>();
    std::unique_ptr<Block::LinearSolverType> linearsolver(
        new g2o::LinearSolverDense<Block::PoseMatrixType>());

    // step 2: 创建Blocksolver，并用上面定义的线性求解器初始化
    // Block* solver_ptr = new Block( linearSolver );
    std::unique_ptr<Block> solver_ptr(new Block(std::move(linearsolver)));

    // step 3: 创建总求解器solver。并从GN, LM, DogLeg
    // 中选一个，再用上述块求解器BlockSolver初始化
    // g2o::OptimizationAlgorithmLevenberg *solver =
    // new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::OptimizationAlgorithmGaussNewton *solve =
        new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));

    // step 4: 创建终极大boss 稀疏优化器（SparseOptimizer）
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solve);
    optimizer.setVerbose(true);

    // 往图中增加顶点，待优化量
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    // 往图中增加边
    for (int i = 0; i < N; i++) {
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(y_data[i]);

        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 /
                             (w_sigma * w_sigma)); // 信息矩阵：协方差矩阵之逆
        optimizer.addEdge(edge);
    }

    // 执行优化
    cout << "start optimization" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used =
        chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "estimated model: " << abc_estimate.transpose() << endl;

    return 0;
}
