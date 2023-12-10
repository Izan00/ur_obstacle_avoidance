#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

double canonical_system_alpha(double goal_z, double goal_t, double start_t, double int_dt = 0.001) {
    if (goal_z <= 0.0) {
        throw std::invalid_argument("Final phase must be > 0!");
    }
    if (start_t >= goal_t) {
        throw std::invalid_argument("Goal must be chronologically after start!");
    }

    double execution_time = goal_t - start_t;
    int n_phases = static_cast<int>(execution_time / int_dt) + 1;
    // assert that the execution_time is approximately divisible by int_dt
    assert(std::abs(((n_phases - 1) * int_dt) - execution_time) < 0.05);
    return (1.0 - std::pow(goal_z, 1.0 / (n_phases - 1))) * (n_phases - 1);
}

double phase(double t, double alpha, double goal_t, double start_t, double int_dt = 0.001, double eps = 1e-10) {
    double execution_time = goal_t - start_t;
    double b = std::max(1.0 - alpha * int_dt / execution_time, eps);
    return std::pow(b, (t - start_t) / int_dt);
}


class ForcingTerm {
public:
    ForcingTerm(int n_dims, int n_weights_per_dim, double goal_t, double start_t, double overlap, double alpha_z) 
        : n_weights_per_dim(n_weights_per_dim), goal_t(goal_t), start_t(start_t), overlap(overlap), alpha_z(alpha_z) {
        if (n_weights_per_dim <= 1) {
            throw std::invalid_argument("The number of weights per dimension must be > 1!");
        }
        if (start_t >= goal_t) {
            throw std::invalid_argument("Goal must be chronologically after start!");
        }
        init_rbfs(n_dims, n_weights_per_dim, start_t);
    }

    void init_rbfs(int n_dims, int n_weights_per_dim, double start_t) {
        log_overlap = -std::log(overlap);
        execution_time = goal_t - start_t;
        weights_ = MatrixXd::Zero(n_dims, n_weights_per_dim);
        centers = VectorXd(n_weights_per_dim);
        widths = VectorXd(n_weights_per_dim);

        double step = execution_time / (n_weights_per_dim - 1);
        double t = start_t;
        centers[0] = phase(t, alpha_z, goal_t, start_t);
        
        for (int i = 1; i < n_weights_per_dim; ++i) {
            t = i * step;
            centers[i] = phase(t, alpha_z, goal_t, start_t);
            double diff = centers[i] - centers[i - 1];
            widths[i - 1] = log_overlap / (diff * diff);
        }
        
        widths[n_weights_per_dim - 1] = widths[n_weights_per_dim - 2];
    }

    MatrixXd activations(const VectorXd& z) {
        MatrixXd squared_dist = (z - centers.replicate(z.size(), 1)).array().pow(2);
        MatrixXd activations = (-widths.replicate(1, z.size()) * squared_dist).array().exp();
        activations.rowwise() /= activations.colwise().sum();
        return activations;
    }

    MatrixXd design_matrix(const VectorXd& T, double int_dt = 0.001) {
        VectorXd Z = phase(T, alpha_z, T[T.size() - 1], T[0], int_dt);
        return Z.transpose().array() * activations(Z).array();
    }

    double phase(double t, double int_dt = 0.001) {
        return phase(t, alpha_z, goal_t, start_t, int_dt);
    }

    MatrixXd forcing_term(const VectorXd& z) {
        MatrixXd activations_matrix = activations(z);
        return z.replicate(1, z.size()).array() * weights_ * activations_matrix;
    }

    VectorXd operator()(double t, double int_dt = 0.001) {
        return forcing_term(VectorXd::Constant(1, phase(t, int_dt)));
    }

    Eigen::MatrixXd weights() const {
        return weights_;
    }

    Eigen::Vector2i shape() const {
        return weights_.rows(), weights_.cols();
    }

private:
    int n_weights_per_dim;
    double goal_t;
    double start_t;
    double overlap;
    double alpha_z;
    double log_overlap;
    double execution_time;
    MatrixXd weights_;
    VectorXd centers;
    VectorXd widths;
};

class WeightParametersMixin {
public:
    Eigen::VectorXd get_weights() const {
        return forcing_term.weights().array().reshape(1, forcing_term.shape().prod());
    }

    void set_weights(const Eigen::VectorXd& weights) {
        int n_pos_weights = forcing_term.weights().size();
        forcing_term.weights() = weights.head(n_pos_weights).array().reshape(forcing_term.shape());
    }

    int n_weights() const {
        return forcing_term.shape().prod();
    }

protected:
    ForcingTerm forcing_term;
};



int main() {
    // Example usage
    double goal_z = 0.5;
    double goal_t = 1.0;
    double start_t = 0.0;

    try {
        double alpha = canonical_system_alpha(goal_z, goal_t, start_t);
        double result = phase(0.5, alpha, goal_t, start_t);
        std::cout << "Result: " << result << std::endl;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}
