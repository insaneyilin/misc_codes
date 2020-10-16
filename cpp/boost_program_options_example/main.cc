#include <iostream>
#include <vector>
#include <string>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

bool ParseArgs(int argc, char **argv,
               po::variables_map *boost_args) {
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "print help message")
      ("compression", po::value<int>()->default_value(3),
           "set compression level")
      ("input-file", po::value<std::string>()->required(), "input file")
      ("output-file", po::value<std::string>(), "compressed output file");

  po::parsed_options parsed_opts = po::parse_command_line(argc, argv, desc);
  po::store(parsed_opts, *boost_args);
  if (boost_args->count("help")) {
    std::cout << desc << '\n';
    return false;
  }

  try {
    po::notify(*boost_args);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
    std::cout << desc << '\n';
    return false;
  }

  return true;
}

int main(int argc, char **argv) {
  boost::program_options::variables_map args;
  if (!ParseArgs(argc, argv, &args)) {
    return 1;
  }

  const std::string input_file = args["input-file"].as<std::string>();
  const std::string output_file = args["output-file"].as<std::string>();
  std::cout << "input: " << input_file << '\n';
  std::cout << "output: " << output_file << '\n';
  std::cout << "compression_level: " << args["compression"].as<int>() << '\n';

  return 0;
}

