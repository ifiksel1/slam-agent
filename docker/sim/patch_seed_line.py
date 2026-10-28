"""Stop SUPER aborting on long straight path segments ("seed line too long").

SUPER's corridor search (corridor_generator.cpp, SearchPolytopeOnPath) grows each seed line along
the front-end path and throws std::runtime_error("seed line too long") when one is longer than
1.5 x corridor_line_max_length. A straight, unobstructed path can reach it with only its two end
points (a 5.1 m vertical climb: start and goal), so the first step is already too long and the
whole fsm_node aborts. This splits every path segment into pieces no longer than the seed-line
limit before the search, so seed lines grow and stop as intended.

Applied at image build time (docker/sim/Dockerfile); SUPER's own checkout stays untouched.
usage: patch_seed_line.py <corridor_generator.cpp>
"""
import sys

path = sys.argv[1]
src = open(path).read()

SIG = "    CorridorGenerator::SearchPolytopeOnPath(const vec_Vec3f &path, PolytopeVec &sfcs,"
NEW_SIG = "    CorridorGenerator::SearchPolytopeOnPath(const vec_Vec3f &path_in, PolytopeVec &sfcs,"
BODY = "        sfcs.clear();\n        if (path.empty()) {"
NEW_BODY = '''        sfcs.clear();
        // split long segments: a straight path with sparse points (a vertical climb: start and goal
        // only) otherwise yields a first seed line over the limit below and aborts fsm_node
        vec_Vec3f path;
        for (size_t i = 0; i < path_in.size(); i++) {
            if (i > 0) {
                const Vec3f d = path_in[i] - path_in[i - 1];
                const int n = static_cast<int>(std::ceil(d.norm() / seed_line_max_length_));
                for (int k = 1; k < n; k++) {
                    path.push_back(path_in[i - 1] + d * (static_cast<double>(k) / n));
                }
            }
            path.push_back(path_in[i]);
        }
        if (path.empty()) {'''

for anchor, new in ((SIG, NEW_SIG), (BODY, NEW_BODY)):
    if src.count(anchor) != 1:
        sys.exit("patch_seed_line: anchor not found exactly once: %r" % anchor[:60])
    src = src.replace(anchor, new)
open(path, "w").write(src)
print("patch_seed_line: applied to", path)
