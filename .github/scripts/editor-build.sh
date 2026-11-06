#!/bin/bash
# Installs and builds the editor checked out in $1 (default editor) for the end-to-end job: production
# mode without minification, source maps or module concatenation, about a minute. The job caches the
# result by the editor commit and this file's hash.
set -euo pipefail
cd "${1:-editor}"
npm ci --ignore-scripts
npm run setup:strucpp
node node_modules/electron/install.js
flags='--no-optimization-minimize --no-devtool --no-optimization-concatenate-modules'
npx concurrently --kill-others-on-fail --names main,renderer \
    "npm run build:main -- $flags" \
    "npm run build:renderer -- $flags"
