#!/usr/bin/env bash
# Rename the beacon label "gnss=" -> "gnss_rx_mode=" in already-decoded
# packet rows, so old rows match the new beacon_cts1.c interpretation
# without re-decoding every pass.
set -euo pipefail

# Resolve the DB path the same way packet_db_default_path() does.
if [[ -n "${SSO_PACKET_DB:-}" ]]; then
    db="$SSO_PACKET_DB"
elif [[ -n "${FRONTIERSAT_ROOT:-}" ]]; then
    db="$FRONTIERSAT_ROOT/packet_db.sqlite"
else
    db="/FrontierSat/packet_db.sqlite"
fi
# Allow an explicit override: migrate_gnss_label.sh /path/to/packet_db.sqlite
db="${1:-$db}"

[[ -f "$db" ]] || { echo "no DB at $db" >&2; exit 1; }
echo "DB: $db"

# Cheap insurance — this is a one-way text mutation.
cp -p "$db" "$db.bak-gnss-label"
echo "backup: $db.bak-gnss-label"

before=$(sqlite3 "$db" \
  "SELECT count(*) FROM packet WHERE decoded_summary LIKE '% gnss=%';")
echo "rows with ' gnss=' before: $before"

sqlite3 "$db" \
  "UPDATE packet
      SET decoded_summary = REPLACE(decoded_summary, ' gnss=', ' gnss_rx_mode=')
    WHERE decoded_summary LIKE '% gnss=%';"

after=$(sqlite3 "$db" \
  "SELECT count(*) FROM packet WHERE decoded_summary LIKE '% gnss=%';")
echo "rows with ' gnss=' after:  $after   (expect 0)"

