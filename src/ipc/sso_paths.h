// sso_paths.h — resolve the shared FrontierSat data root and its subdirs.
//
// The ground machine stores per-pass artifacts, TLEs, the packet DB and
// the satnogs archive under /FrontierSat/. Dev hosts fall back to
// $HOME/FrontierSat so the same code paths work everywhere. The
// resolution order is documented at sso_frontiersat_root().

#ifndef SSO_PATHS_H
#define SSO_PATHS_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Absolute path of the FrontierSat data root.
// Resolution order:
//   1. $FRONTIERSAT_ROOT if set and non-empty.
//   2. /FrontierSat if it exists and is a directory.
//   3. $HOME/FrontierSat (dev-host fallback, may not yet exist).
// Returned pointer is valid until the next call. Not thread-safe; first
// caller wins for the process lifetime in practice.
const char *sso_frontiersat_root(void);

// Convenience wrappers for the standard subdirs. Each returns an
// absolute path that lives in the per-call static buffer (not
// thread-safe). The directory may not exist yet — callers create it
// when they intend to write.
const char *sso_tles_dir(void);
const char *sso_operations_dir(void);
const char *sso_operations_current_symlink(void);
const char *sso_satnogs_archive_dir(void);
const char *sso_captures_dir(void);
const char *sso_packet_db_path(void);
// Bench / characterisation captures live under a sibling tree so they
// don't fight the pass-folder layout under Operations/. Same yyyymmdd /
// hhmmLT date layering, but the timestamp is "now" (the moment the
// operator hit start), not a predicted AOS.
const char *sso_testing_dir(void);

// Compose <root>/<subdir> into the caller's buffer. Returns 0 on
// success, -1 on overflow. <subdir> may include further slashes.
int sso_frontiersat_subpath(char *out, size_t out_size, const char *subdir);

// mkdir -p semantics. Returns 0 on success / already-exists, -1 on
// error (with errno set). sso_mkdir_p_for_file creates the parent
// directory of `path`; sso_mkdir_p creates `path` itself.
int sso_mkdir_p(const char *path);
int sso_mkdir_p_for_file(const char *path);

#ifdef __cplusplus
}
#endif

#endif
