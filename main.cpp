#include <argparse/argparse.hpp>
#include <lacam.hpp>

int main(int argc, char* argv[])
{
  // arguments parser
  argparse::ArgumentParser program("lacam", "0.1.0");
  program.add_argument("-m", "--map").help("map file").required();
  program.add_argument("-i", "--scen")
      .help("scenario file")
      .default_value(std::string(""));
  program.add_argument("-N", "--num").help("number of agents").required();
  program.add_argument("-s", "--seed")
      .help("seed")
      .default_value(std::string("0"));
  program.add_argument("-v", "--verbose")
      .help("verbose")
      .default_value(std::string("0"));
  program.add_argument("-t", "--time_limit_sec")
      .help("time limit sec")
      .default_value(std::string("3"));
  program.add_argument("-o", "--output")
      .help("output file")
      .default_value(std::string("./build/result.txt"));
  program.add_argument("-l", "--log_short")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("-O", "--objective")
      .help("0: makespan, 1: sum_of_loss")
      .default_value(std::string("0"))
      .action([](const std::string& value) {
        static const std::vector<std::string> C = {"0", "1", "2"};
        if (std::find(C.begin(), C.end(), value) != C.end()) return value;
        return std::string("0");
      });
  program.add_argument("-r", "--restart_rate")
      .help("restart rate")
      .default_value(std::string("0.001"));

  try {
    program.parse_known_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  // setup instance
  const auto verbose = std::stoi(program.get<std::string>("verbose"));
  const auto time_limit_sec =
      std::stoi(program.get<std::string>("time_limit_sec"));
  const auto scen_name = program.get<std::string>("scen");
  const auto seed = std::stoi(program.get<std::string>("seed"));
  auto MT = std::mt19937(seed);
  const auto map_name = program.get<std::string>("map");
  const auto output_name = program.get<std::string>("output");
  const auto log_short = program.get<bool>("log_short");
  const auto N = std::stoi(program.get<std::string>("num"));
  const auto ins = scen_name.size() > 0 ? Instance(scen_name, map_name, N)
                                        : Instance(map_name, &MT, N);
  const auto objective =
      static_cast<Objective>(std::stoi(program.get<std::string>("objective")));
  const auto restart_rate = std::stof(program.get<std::string>("restart_rate"));
  if (!ins.is_valid(1)) return 1;

  // solve
  auto additional_info = std::string("");
  const auto deadline = Deadline(time_limit_sec * 1000);
  const auto solution = solve(ins, additional_info, verbose - 1, &deadline, &MT,
                              objective, restart_rate);
  const auto comp_time_ms = deadline.elapsed_ms();

  // failure
  if (solution.empty()) info(1, verbose, "failed to solve");

  // check feasibility
  if (!is_feasible_solution(ins, solution, verbose)) {
    info(0, verbose, "invalid solution");
  }

  // post processing
  print_stats(verbose, ins, solution, comp_time_ms);
  make_log(ins, solution, output_name, comp_time_ms, map_name, seed,
           additional_info, log_short);
  return 0;
}
