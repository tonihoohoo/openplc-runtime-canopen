#!/bin/bash
# Writes docs_only=true to $GITHUB_OUTPUT when the pull request changes only Markdown files or
# files under docs/, so the test jobs can be skipped. Any other event writes docs_only=false.
set -euo pipefail

docs_only=false
if [ "${GITHUB_EVENT_NAME:-}" = "pull_request" ]; then
    files=$(gh api --paginate "repos/${GITHUB_REPOSITORY}/pulls/${PR_NUMBER}/files" --jq '.[].filename')
    if [ -n "$files" ]; then
        docs_only=true
        while IFS= read -r file; do
            case "$file" in
                *.md | docs/*) ;;
                *) docs_only=false ;;
            esac
        done <<<"$files"
    fi
    echo "Changed files:"
    echo "$files"
fi
echo "docs_only=$docs_only"
echo "docs_only=$docs_only" >>"${GITHUB_OUTPUT:-/dev/stdout}"
