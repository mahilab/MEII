#include <MEII/EMG/EmgFeatures.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

namespace meii {

    double mean_rms(const std::vector<double>& mes_window) {
        double sum_squares = 0.0;
        for (int i = 0; i < mes_window.size(); ++i) {
            sum_squares += std::pow(mes_window[i], 2);
        }
        return std::sqrt(sum_squares / mes_window.size());
    }

    double mean_absolute_value(const std::vector<double>& mes_window) {
        double sum_abs = 0.0;
        for (int i = 0; i < mes_window.size(); ++i) {
            sum_abs += std::abs(mes_window[i]);
        }
        return sum_abs / mes_window.size();
    }

    double wavelength(const std::vector<double>& mes_window) {
        double sum_abs_diff = 0.0;
        for (int i = 0; i < mes_window.size() - 1; ++i) {
            sum_abs_diff += std::abs(mes_window[i + 1] - mes_window[i]);
        }
        return sum_abs_diff;
    }

    double zero_crossings(const std::vector<double>& mes_window) {
        double sum_abs_diff_sign = 0.0;
        for (int i = 0; i < mes_window.size() - 1; ++i) {
            sum_abs_diff_sign += std::abs(std::copysign(1.0, mes_window[i + 1]) - std::copysign(1.0, mes_window[i]));
        }
        return sum_abs_diff_sign / 2.0;
    }

    double slope_sign_changes(const std::vector<double>& mes_window) {
        double sum_abs_diff_sign_diff = 0.0;
        for (int i = 0; i < mes_window.size() - 2; ++i) {
            sum_abs_diff_sign_diff += std::abs(std::copysign(1.0, (mes_window[i + 2] - mes_window[i + 1])) - std::copysign(1.0, (mes_window[i + 1] - mes_window[i])));
        }
        return sum_abs_diff_sign_diff / 2.0;
    }

    void auto_regressive_coefficients(std::vector<double>& coeffs, const std::vector<double>& mes_window) {

        // Algorithm taken from Cedrick Collomb, "Burg's Method, Algorithm and Recursion," November 8, 2009

        // initialize
        std::size_t N = mes_window.size() - 1;
        std::size_t m = coeffs.size();
        std::vector<double> A_k(m + 1, 0.0);
        A_k[0] = 1.0;
        std::vector<double> f = mes_window;
        std::vector<double> b = mes_window;

        double D_k = 0;
        for (std::size_t j = 0; j <= N; ++j) {
            D_k += 2.0 * f[j] * f[j];
        }
        D_k -= f[0] * f[0] + b[N] * b[N];


        // Burg recursion
        for (std::size_t k = 0; k < m; ++k) {

            // compute mu
            double mu = 0.0;
            for (std::size_t n = 0; n <= N - k - 1; ++n) {
                mu += f[n + k + 1] * b[n];
            }
            mu *= -2.0 / D_k;

            // update A_k
            for (std::size_t n = 0; n <= (k + 1) / 2; ++n) {
                double t1 = A_k[n] + mu * A_k[k + 1 - n];
                double t2 = A_k[k + 1 - n] + mu * A_k[n];
                A_k[n] = t1;
                A_k[k + 1 - n] = t2;
            }

            // update f and b
            for (std::size_t n = 0; n <= N - k - 1; ++n) {
                double t1 = f[n + k + 1] + mu * b[n];
                double t2 = b[n] + mu * f[n + k + 1];
                f[n + k + 1] = t1;
                b[n] = t2;
            }

            // update D_k
            D_k = (1.0 - mu * mu) * D_k - f[k + 1] * f[k + 1] - b[N - k - 1] * b[N - k - 1];
        }

        // assign coefficients
        coeffs.assign(++A_k.begin(), A_k.end());
    }

} // namespace meii