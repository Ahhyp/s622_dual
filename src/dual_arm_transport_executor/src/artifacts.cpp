#include "dual_arm_transport_executor/artifacts.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace dual_arm_transport_executor
{

const char * toString(Protection p)
{
  switch (p) {
    case Protection::None: return "none";
    case Protection::Hold: return "hold";
    case Protection::Fault: return "fault";
    case Protection::Abort: return "abort";
  }
  return "unknown";
}

namespace
{

bool writeFile(const std::string & path, const std::string & body, std::string * err)
{
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f) {
    if (err) {*err = "cannot open for writing: " + path;}
    return false;
  }
  f << body;
  if (!f) {
    if (err) {*err = "write failed: " + path;}
    return false;
  }
  return true;
}

std::string escapeJson(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

std::string num(double v)
{
  if (!(v == v) || v == v * 2.0) {  // NaN / Inf are not valid JSON
    return "null";
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

std::string csvCell(const std::string & s)
{
  if (s.find_first_of(",\"\n") == std::string::npos) {return s;}
  std::string out = "\"";
  for (char c : s) {
    if (c == '"') {out += "\"\"";} else {out += c;}
  }
  out += "\"";
  return out;
}

}  // namespace

bool ArtifactWriter::open(const std::string & dir, std::string * err)
{
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    if (err) {*err = "cannot create artifact dir: " + dir + ": " + ec.message();}
    return false;
  }
  dir_ = dir;
  return true;
}

bool ArtifactWriter::writeManifest(const Manifest & m, std::string * err) const
{
  std::ostringstream o;
  o << "{\n";
  o << "  \"schema\": \"c43_s4_w3r1_manifest/v1\",\n";
  o << "  \"run_id\": \"" << escapeJson(m.run_id) << "\",\n";
  o << "  \"git_head\": \"" << escapeJson(m.git_head) << "\",\n";
  o << "  \"package_version\": \"" << escapeJson(m.package_version) << "\",\n";
  o << "  \"build_type\": \"" << escapeJson(m.build_type) << "\",\n";
  o << "  \"mode\": \"" << escapeJson(m.mode) << "\",\n";
  o << "  \"dry_run\": " << (m.dry_run ? "true" : "false") << ",\n";
  o << "  \"env_class\": \"" << escapeJson(m.env_class) << "\",\n";
  o << "  \"input_config_sha256\": \"" << escapeJson(m.input_config_sha256) << "\",\n";
  o << "  \"started_utc\": \"" << escapeJson(m.started_utc) << "\"\n";
  o << "}\n";
  return writeFile(dir_ + "/" + ArtifactNames::manifest(), o.str(), err);
}

bool ArtifactWriter::writeEvents(const std::vector<Event> & events, std::string * err) const
{
  std::ostringstream o;
  o << "t,kind,from,to,code,detail\n";
  for (const auto & e : events) {
    o << num(e.t) << ',' << csvCell(e.kind) << ',' << csvCell(e.from) << ','
      << csvCell(e.to) << ',' << csvCell(e.code) << ',' << csvCell(e.detail) << '\n';
  }
  return writeFile(dir_ + "/" + ArtifactNames::events(), o.str(), err);
}

bool ArtifactWriter::writeCycles(const std::vector<CycleRow> & rows, std::string * err) const
{
  std::ostringstream o;
  o << "k,t,state,failure_code,committed,t_traj,q_cmd\n";
  for (const auto & r : rows) {
    o << r.k << ',' << num(r.t) << ',' << csvCell(r.state) << ','
      << csvCell(r.failure_code) << ',' << (r.committed ? 1 : 0) << ',' << num(r.t_traj)
      << ",\"";
    for (int i = 0; i < 12; ++i) {
      if (i) {o << ' ';}
      o << num(r.q_cmd[static_cast<size_t>(i)]);
    }
    o << "\"\n";
  }
  return writeFile(dir_ + "/" + ArtifactNames::cycles(), o.str(), err);
}

bool ArtifactWriter::writeSummary(const std::string & json_body, std::string * err) const
{
  return writeFile(dir_ + "/" + ArtifactNames::summary(), json_body, err);
}

bool ArtifactWriter::writeCommands(const std::vector<std::string> & argv, std::string * err) const
{
  std::ostringstream o;
  for (size_t i = 0; i < argv.size(); ++i) {
    if (i) {o << ' ';}
    o << argv[i];
  }
  o << '\n';
  return writeFile(dir_ + "/" + ArtifactNames::commands(), o.str(), err);
}

bool ArtifactWriter::copyTestResults(const std::string & xml_path, std::string * err) const
{
  if (xml_path.empty()) {
    // An explicit placeholder is written instead of silently omitting the file:
    // a missing required artifact must be visible to the checker.
    return writeFile(dir_ + "/" + ArtifactNames::testResults(),
                     "<?xml version=\"1.0\"?>\n<!-- no gtest xml was supplied to this run -->\n"
                     "<testsuites/>\n", err);
  }
  std::ifstream in(xml_path, std::ios::binary);
  if (!in) {
    if (err) {*err = "cannot read gtest xml: " + xml_path;}
    return false;
  }
  std::ostringstream body;
  body << in.rdbuf();
  return writeFile(dir_ + "/" + ArtifactNames::testResults(), body.str(), err);
}

std::string buildSummaryJson(const std::string & run_id, const std::string & result,
                             const std::vector<Event> & events,
                             const std::vector<CycleRow> & cycles,
                             const std::vector<std::string> & extra_lines,
                             bool dry_run,
                             const std::vector<std::string> & phases,
                             const std::string & mode)
{
  // Deliberately excludes wall-clock and temp paths: two identical runs must
  // produce identical bytes.
  int protections = 0;
  int advances = 0;
  int rejects = 0;
  for (const auto & e : events) {
    if (e.kind == "protection") {++protections;}
    if (e.kind == "advance") {++advances;}
    if (e.kind == "reject") {++rejects;}
  }
  std::ostringstream o;
  o << "{\n";
  o << "  \"schema\": \"c43_s4_w3r1_summary/v1\",\n";
  o << "  \"run_id\": \"" << escapeJson(run_id) << "\",\n";
  o << "  \"result\": \"" << escapeJson(result) << "\",\n";
  if (!mode.empty()) {
    o << "  \"mode\": \"" << escapeJson(mode) << "\",\n";
  }
  o << "  \"dry_run\": " << (dry_run ? "true" : "false") << ",\n";
  o << "  \"cycles\": " << cycles.size() << ",\n";
  o << "  \"events\": " << events.size() << ",\n";
  o << "  \"protection_events\": " << protections << ",\n";
  o << "  \"advances\": " << advances << ",\n";
  o << "  \"rejections\": " << rejects << ",\n";
  o << "  \"phases\": [";
  for (size_t i = 0; i < phases.size(); ++i) {
    if (i) {o << ", ";}
    o << "\"" << escapeJson(phases[i]) << "\"";
  }
  o << "],\n";
  o << "  \"final_state\": \"";
  o << (cycles.empty() ? "IDLE" : cycles.back().state) << "\",\n";
  o << "  \"extra\": [";
  for (size_t i = 0; i < extra_lines.size(); ++i) {
    if (i) {o << ", ";}
    o << "\"" << escapeJson(extra_lines[i]) << "\"";
  }
  o << "]\n";
  o << "}\n";
  return o.str();
}

}  // namespace dual_arm_transport_executor
