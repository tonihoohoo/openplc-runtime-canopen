#!/bin/bash
# Writes the changed-file gates to $GITHUB_OUTPUT:
#   docs_only    the pull request changes only Markdown files or files under docs/
#   plugin_only  it changes only the CANopen plugin, the CANopen CI files, Markdown or docs/
#   master, slave, lss  the CANopen end-to-end groups its files need (see canopen.yml)
# Any other event, and a pull request with a same-named editor branch, needs every group.
# Needs GH_TOKEN, GITHUB_REPOSITORY, and on pull requests PR_NUMBER and HEAD_REF; EDITOR_REPOSITORY
# names the editor fork.
set -euo pipefail

plugin=core/src/drivers/plugins/native/canopen
docs_only=false
plugin_only=false
master=true
slave=true
lss=true

# The groups one changed file needs, as words; "all" for every group.
groups() {
    case "$1" in
        *.md | docs/*) ;;
        "$plugin"/tests/*) ;;
        "$plugin"/slave/lss_store.*) echo slave lss ;;
        "$plugin"/slave/*) echo slave ;;
        "$plugin"/lss.cpp | "$plugin"/lss_online.*) echo lss ;;
        "$plugin"/network.* | "$plugin"/client.* | "$plugin"/requests.* | "$plugin"/master_dcf.* | \
            "$plugin"/bus_monitor.* | "$plugin"/bus_off_recovery.* | "$plugin"/link_stall.* | \
            "$plugin"/plc_port.*) echo master ;;
        .github/scripts/canopen_online_e2e.py | .github/scripts/canopen_blocks_e2e.py) echo master ;;
        .github/scripts/canopen_slave_e2e.py | .github/scripts/canopen_slave_blocks_e2e.py | \
            .github/scripts/linux_io_e2e.py | .github/scripts/gpio-sim-up.sh) echo slave ;;
        .github/scripts/canopen_lss_e2e.py | .github/scripts/canopen_lss_out_of_turn_e2e.py | \
            .github/scripts/lss_out_of_turn_device.py | .github/scripts/canopen_slave_lss_e2e.py) echo lss ;;
        .github/workflows/canopen.yml | .github/scripts/docs-only.sh | .github/scripts/vcan-up.sh) echo all ;;
        .github/*) ;;
        *) echo all ;;
    esac
}

# A CANopen CI file under .github/: one the CANopen workflow or the banned-word check uses.
canopen_ci() {
    case "$1" in
        .github/workflows/canopen*.yml | .github/workflows/banned-words.yml) return 0 ;;
        .github/scripts/canopen* | .github/scripts/lss_* | .github/scripts/linux_io_e2e.py) return 0 ;;
        .github/scripts/gpio-sim-up.sh | .github/scripts/vcan-up.sh | .github/scripts/check-banned-words.py) return 0 ;;
        *) return 1 ;;
    esac
}

if [ "${GITHUB_EVENT_NAME:-}" = "pull_request" ]; then
    files=$(gh api --paginate "repos/${GITHUB_REPOSITORY}/pulls/${PR_NUMBER}/files" --jq '.[].filename')
    echo "Changed files:"
    echo "$files"
    if [ -n "$files" ]; then
        docs_only=true
        plugin_only=true
        master=false
        slave=false
        lss=false
        while IFS= read -r file; do
            case "$file" in
                *.md | docs/*) ;;
                *) docs_only=false ;;
            esac
            case "$file" in
                *.md | docs/* | "$plugin"/*) ;;
                *) canopen_ci "$file" || plugin_only=false ;;
            esac
            for g in $(groups "$file"); do
                case "$g" in
                    all) master=true slave=true lss=true ;;
                    master) master=true ;;
                    slave) slave=true ;;
                    lss) lss=true ;;
                esac
            done
        done <<<"$files"
    fi
    if [ -n "${EDITOR_REPOSITORY:-}" ] && [ -n "${HEAD_REF:-}" ] &&
        git ls-remote --exit-code --heads "https://github.com/$EDITOR_REPOSITORY" "$HEAD_REF" >/dev/null 2>&1; then
        echo "The editor has a branch $HEAD_REF: every end-to-end group runs."
        master=true slave=true lss=true
    fi
fi
{
    echo "docs_only=$docs_only"
    echo "plugin_only=$plugin_only"
    echo "master=$master"
    echo "slave=$slave"
    echo "lss=$lss"
} | tee -a "${GITHUB_OUTPUT:-/dev/stdout}"
