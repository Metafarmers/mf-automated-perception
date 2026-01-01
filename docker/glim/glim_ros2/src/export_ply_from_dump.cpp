#include <iostream>
#include <memory>
#include <string>

#include <boost/program_options.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>

#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>

#include <glk/io/ply_io.hpp>

int main(int argc, char** argv) {
  using namespace boost::program_options;

  // --------------------------------------------------
  // CLI options
  // --------------------------------------------------
  options_description desc("GLIM dump exporter");
  desc.add_options()
    ("help,h", "show help message")
    ("dump_path", value<std::string>(), "Input GLIM dump directory")
    ("output,o", value<std::string>(), "Output PLY path")
    ("config_path,c", value<std::string>()->default_value("config"),
      "Config path (absolute or relative to glim share)")
    ("debug,d", "Enable debug logging");

  positional_options_description po;
  po.add("dump_path", 1);
  po.add("output", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(desc).positional(po).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("dump_path") || !vm.count("output")) {
    std::cout << desc << std::endl;
    return 0;
  }

  const std::string dump_path = vm["dump_path"].as<std::string>();
  const std::string ply_path  = vm["output"].as<std::string>();

  // --------------------------------------------------
  // Logger setup (same policy as offline_viewer)
  // --------------------------------------------------
  auto logger = spdlog::stdout_color_mt("glim");
  logger->sinks().push_back(glim::get_ringbuffer_sink());

  if (vm.count("debug")) {
    logger->sinks().push_back(
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        "/tmp/export_ply_log.log", true));
    logger->set_level(spdlog::level::trace);
  }

  spdlog::set_default_logger(logger);

  // --------------------------------------------------
  // Config path resolution (offline_viewer compatible)
  // --------------------------------------------------
  std::string config_path = vm["config_path"].as<std::string>();
  if (!config_path.empty() && config_path[0] != '/') {
    config_path =
      ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  spdlog::info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);

  // --------------------------------------------------
  // Global mapping setup
  // --------------------------------------------------
  glim::GlobalMappingParams params;
  params.enable_optimization = false;
  params.isam2_relinearize_skip = 1;
  params.isam2_relinearize_thresh = 0.0;

  auto global_mapping = std::make_shared<glim::GlobalMapping>(params);

  if (!global_mapping->load(dump_path)) {
    spdlog::error("failed to load dump from {}", dump_path);
    return 1;
  }

  glim::AsyncGlobalMapping async_mapping(global_mapping, 1e6);

  // --------------------------------------------------
  // Export points
  // --------------------------------------------------
  auto points = async_mapping.export_points();
  if (!points || !points->has_points()) {
    spdlog::warn("no points available for export");
    return 1;
  }

  glk::PLYData ply;
  ply.vertices.reserve(points->size());

  const bool has_intensities = points->has_intensities();
  if (has_intensities) {
    ply.intensities.reserve(points->size());
  }

  for (size_t i = 0; i < points->size(); ++i) {
    ply.vertices.push_back(points->points[i].head<3>().cast<float>());
    if (has_intensities) {
      ply.intensities.push_back(points->intensities[i]);
    }
  }

  glk::save_ply_binary(ply_path, ply);

  spdlog::info("exported {} points to {}", ply.vertices.size(), ply_path);
  return 0;
}

