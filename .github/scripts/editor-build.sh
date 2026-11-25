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
# The steps of `npm run build`, with the flags on the webpack runs only and without package-trust
# verify-bundle, which needs a minified bundle.
npm run build:hmi-objects
main="TS_NODE_TRANSPILE_ONLY=true npx ts-node scripts/prepare-yowasp-clang.ts \
&& npx ts-node --transpile-only scripts/package-trust/cli.ts prepare \
&& NODE_ENV=production TS_NODE_TRANSPILE_ONLY=true npx webpack --config ./configs/webpack/webpack.config.main.prod.ts $flags"
npx concurrently --kill-others-on-fail --names main,renderer \
    "$main" \
    "npm run build:renderer -- $flags"
