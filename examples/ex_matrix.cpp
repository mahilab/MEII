#include <MEII/Utility/Matrix.hpp>
#include <MEL/Core/Console.hpp>

using namespace mel;
using namespace meii;

int main() {

    // Matrix construction and element assignment
    Matrix A(2, 2);
    A(0, 0) = 1.0;
    A(0, 1) = 2.0;
    A(1, 0) = 3.0;
    A(1, 1) = 4.0;
    print("Matrix A = ");
    std::cout << A << std::endl;

    // Matrix simple assignment
    Matrix B = A;
    print("Matrix B = A = ");
    std::cout << B << std::endl;

    // Matrix scalar addition
    B = B + 2.0;
    print("B = B + 2.0 = ");
    std::cout << B << std::endl;

    // Matrix scalar subtraction
    B -= 2.0;
    print("B -= 2.0 = ");
    std::cout << B << std::endl;

    // Matrix addition
    Matrix C = A + B;
    print("Matrix C = A + B = ");
    std::cout << C << std::endl;

    // Matrix subtraction
    C -= B;
    print("C -= B = ");
    std::cout << C << std::endl;

    // Square matrix multiplication
    C = A * B;
    print("C = A * B = ");
    std::cout << C << std::endl;

    // Square matrix self-multiplication
    A *= A;
    print("A *= A = ");
    std::cout << A << std::endl;

    // Matrix vector multiplication
    std::vector<double> x = { 4, 5 };
    print("vector x = ");
    std::cout << x << std::endl << std::endl;
    x = A * x;
    print("vector x = A * x = ");
    std::cout << x << std::endl << std::endl;

    // Non-square matrix multiplication
    Matrix D(2, 3);
    D(0, 0) = 1.0;
    D(0, 1) = 2.0;
    D(0, 2) = 3.0;
    D(1, 0) = 4.0;
    D(1, 1) = 5.0;
    D(1, 2) = 6.0;
    Matrix E(3, 2);
    E(0, 0) = 0.1;
    E(0, 1) = 0.2;
    E(1, 0) = 0.1;
    E(1, 1) = 0.2;
    E(2, 0) = 0.1;
    E(2, 1) = 0.2;
    print("Matrix D = ");
    std::cout << D << std::endl;
    print("Matrix E = ");
    std::cout << E << std::endl;
    A = D * E;
    print("A = D * E");
    std::cout << A << std::endl;

    return 0;
}