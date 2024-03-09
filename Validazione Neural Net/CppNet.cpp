#include <iostream>
#include "../packages/eigen-3.4.0/Eigen/Geometry"
#include <fstream>

// To run: g++ CppNet.cpp -o CppNet && ./CppNet

template <typename M>
M load_csv(const std::string &path)
{
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<double> values;
  uint rows = 0;
  while (std::getline(indata, line))
  {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ','))
    {
      values.push_back(std::stod(cell));
    }
    ++rows;
  }
  return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size() / rows);
}

std::vector<Eigen::MatrixXd> W;
std::vector<Eigen::MatrixXd> B;

Eigen::VectorXd neuralNetwork(const Eigen::VectorXd x)
{

  Eigen::VectorXd output = x;

  for (int i = 0; i < W.size(); i++)
  {
    output = W[i] * output;
    for (int j = 0; j < B[i].size(); j++)
    {
      output(j) = output(j) + B[i](j);
      if (i < (W.size() - 1) && output(j) < 0)
        output(j) = output(j) * 0.01;
    }
  }
  return output;
}

int main()
{
  W.reserve(2);
  B.reserve(2);
  W.push_back(load_csv<Eigen::MatrixXd>("./Weights/W1.csv"));
  W.push_back(load_csv<Eigen::MatrixXd>("./Weights/W2.csv"));
  B.push_back(load_csv<Eigen::MatrixXd>("./Weights/B1.csv"));
  B.push_back(load_csv<Eigen::MatrixXd>("./Weights/B2.csv"));

  Eigen::Matrix<double, 9, 1> sample_state;
  sample_state << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  Eigen::Matrix<double, 4, 1> output = neuralNetwork(sample_state);
  std::cout << "Output_nn:\n"
            << output << std::endl;

  return 0;
}
