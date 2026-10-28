// C4.3 S4 / W2-C3 D2-B: independent fixture generator (isolated tool, not in the build).
//
// Purpose
// -------
// The C3 criterion still compares against a *frozen* S3 fixture
// (`src/dual_arm_qp_controller/test/fixtures/s3_ff_1_dq.csv`, 29 columns).
// That fixture was cut from the S3 run artifact `runs/c43_s3/w8/ff_1/cycles.csv`
// (44 columns).  D2-B replaces the *comparison baseline* (same build, frozen
// regression fixture), which changes the acceptance semantics; before such a
// baseline may be frozen, the derivation S3-artifact -> fixture must be
// reproducible by a tool that shares no code with the QP library.
//
// This tool shares nothing with `dual_arm_qp_core`: it is a standalone C++17
// text transform.  It copies the numeric tokens verbatim (no reformatting, no
// re-rounding), so a faithful derivation reproduces the artifact's values
// bit-for-bit, including the exact decimal spellings.
//
// Safety
// ------
// * The output path is REQUIRED and must be given explicitly; there is no
//   default that could overwrite the frozen fixture by accident.
// * Writing over `src/dual_arm_qp_controller/test/fixtures/s3_ff_1_dq.csv`
//   additionally requires `--allow-frozen-overwrite` (D2-B approval).
// * Exit codes: 0 ok, 1 usage, 2 malformed input, 3 refused overwrite.

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr const char * kFrozenMarker = "test/fixtures/s3_ff_1_dq.csv";

std::vector<std::string> split(const std::string & line, char sep) {
  std::vector<std::string> out;
  std::string cell;
  std::istringstream is(line);
  while (std::getline(is, cell, sep)) out.push_back(cell);
  return out;
}

std::vector<std::string> splitWs(const std::string & field) {
  std::vector<std::string> out;
  std::istringstream is(field);
  std::string tok;
  while (is >> tok) out.push_back(tok);
  return out;
}

int usage(const char * argv0) {
  std::cerr << "usage: " << argv0
            << " --artifact <path> --out <path> [--allow-frozen-overwrite]\n"
               "  rewrites the 44-column S3 cycle artifact as the 29-column\n"
               "  fixture (k,t,q_prev[12],dq[12],e_obj_pos_m,e_obj_rot_rad,committed)\n";
  return 1;
}

}  // namespace

int main(int argc, char ** argv) {
  std::string artifact;
  std::string out_path;
  bool allow_frozen = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--artifact") == 0 && i + 1 < argc) {
      artifact = argv[++i];
    } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
      out_path = argv[++i];
    } else if (std::strcmp(argv[i], "--allow-frozen-overwrite") == 0) {
      allow_frozen = true;
    } else {
      return usage(argv[0]);
    }
  }
  if (artifact.empty() || out_path.empty()) return usage(argv[0]);

  // Refuse to touch the frozen fixture unless the caller says so explicitly.
  if (!allow_frozen && out_path.find(kFrozenMarker) != std::string::npos) {
    std::cerr << "refusing to overwrite the frozen fixture: " << out_path
              << "\n  (D2-B approval required; pass --allow-frozen-overwrite)\n";
    return 3;
  }

  std::ifstream in(artifact);
  if (!in) {
    std::cerr << "cannot open artifact: " << artifact << "\n";
    return 2;
  }

  std::string header_line;
  if (!std::getline(in, header_line)) {
    std::cerr << "empty artifact\n";
    return 2;
  }
  const std::vector<std::string> header = split(header_line, ',');
  auto index_of = [&header](const std::string & name) -> int {
    for (std::size_t i = 0; i < header.size(); ++i) {
      if (header[i] == name) return static_cast<int>(i);
    }
    return -1;
  };
  const int i_k = index_of("k");
  const int i_t = index_of("t");
  const int i_committed = index_of("committed");
  const int i_q_prev = index_of("q_prev");
  const int i_dq = index_of("dq");
  const int i_e_pos = index_of("e_obj_pos_m");
  const int i_e_rot = index_of("e_obj_rot_rad");
  if (i_k < 0 || i_t < 0 || i_committed < 0 || i_q_prev < 0 || i_dq < 0 ||
      i_e_pos < 0 || i_e_rot < 0) {
    std::cerr << "artifact is missing one of the required columns\n";
    return 2;
  }

  std::ostringstream out;
  out << "k,t";
  for (int i = 0; i < 12; ++i) out << ",q_prev" << i;
  for (int i = 0; i < 12; ++i) out << ",dq" << i;
  out << ",e_obj_pos_m,e_obj_rot_rad,committed\n";

  std::string line;
  int rows = 0;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const std::vector<std::string> f = split(line, ',');
    const std::size_t need = static_cast<std::size_t>(
        std::max({i_k, i_t, i_committed, i_q_prev, i_dq, i_e_pos, i_e_rot}));
    if (f.size() <= need) {
      std::cerr << "row " << rows << " has only " << f.size() << " columns\n";
      return 2;
    }
    const std::vector<std::string> q_prev = splitWs(f[i_q_prev]);
    const std::vector<std::string> dq = splitWs(f[i_dq]);
    if (q_prev.size() != 12 || dq.size() != 12) {
      std::cerr << "row " << rows << " : q_prev has " << q_prev.size()
                << " and dq has " << dq.size() << " values, expected 12 each\n";
      return 2;
    }
    out << f[i_k] << ',' << f[i_t];
    for (const std::string & tok : q_prev) out << ',' << tok;
    for (const std::string & tok : dq) out << ',' << tok;
    out << ',' << f[i_e_pos] << ',' << f[i_e_rot] << ','
        << (f[i_committed] == "0" ? '0' : '1') << '\n';
    ++rows;
  }
  if (rows == 0) {
    std::cerr << "artifact has no data rows\n";
    return 2;
  }

  std::ofstream os(out_path, std::ios::binary | std::ios::trunc);
  if (!os) {
    std::cerr << "cannot write: " << out_path << "\n";
    return 2;
  }
  os << out.str();
  os.close();
  if (!os) {
    std::cerr << "write failed: " << out_path << "\n";
    return 2;
  }
  std::printf("wrote %d rows to %s\n", rows, out_path.c_str());
  return 0;
}
