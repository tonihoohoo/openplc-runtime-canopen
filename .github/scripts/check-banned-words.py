#!/usr/bin/env python3
"""Fails when a banned word appears in this fork's own commits, branch or pull request.

The words are kept out of the repository: one extended regular expression per
line (case-insensitive, '#' lines and blank lines ignored), read from the file
in $BANNED_WORDS_FILE, else from $BANNED_WORDS (CI passes a repository secret).

Upstream's history is not checked: --commits takes a range such as
origin/main..HEAD, and every commit in it is checked on its own (message,
author, committer, the lines it adds and the paths it touches), so a word
added in one commit and removed in a later one is still found.

Findings name the place and the number of the list line that matched, never
the matched text, so a public CI log does not show the words either.

  check-banned-words.py --commits BASE..HEAD    every commit in the range
  check-banned-words.py --staged                staged changes (pre-commit hook)
  check-banned-words.py --message-file FILE     a commit message (commit-msg hook)
  check-banned-words.py --env-text VAR          text in an environment variable (PR title, body, branch)
  --require-list                                fail when no list is given
"""

import argparse
import os
import re
import subprocess
import sys


def load_patterns():
    path = os.environ.get("BANNED_WORDS_FILE")
    if path:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    elif os.environ.get("BANNED_WORDS", "").strip():
        text = os.environ["BANNED_WORDS"]
    else:
        return None
    patterns = []
    for number, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if line and not line.startswith("#"):
            patterns.append((number, re.compile(line.encode("utf-8"), re.IGNORECASE)))
    return patterns


def git(*args):
    return subprocess.run(["git"] + list(args), check=True, stdout=subprocess.PIPE).stdout


class Checker:
    def __init__(self, patterns):
        self.patterns = patterns
        self.findings = []

    def scan(self, where, data):
        """Records one finding per matching line and list entry."""
        for lineno, line in enumerate(data.split(b"\n"), 1):
            self.scan_line("%s:%d" % (where, lineno), line)

    def scan_line(self, where, line):
        for number, rx in self.patterns:
            if rx.search(line):
                self.findings.append("%s: banned word (list line %d)" % (where, number))

    def scan_patch(self, where, patch, parents):
        """Scans the added lines and the paths of a patch; parents > 1 reads a combined diff."""
        width = max(parents, 1)
        path = "?"
        lineno = 0
        for line in patch.split(b"\n"):
            if line.startswith((b"diff --git ", b"diff --cc ", b"diff --combined ")):
                path = line.split(b" ", 2)[2].decode("utf-8", "replace")
                self.scan_line("%s %s (path)" % (where, path), line)
            elif line.startswith((b"rename to ", b"copy to ", b"+++ ")):
                self.scan_line("%s %s (path)" % (where, path), line)
            elif line.startswith(b"@@"):
                m = re.search(rb"\+(\d+)", line)
                lineno = int(m.group(1)) if m else 0
            elif line.startswith(b"---"):
                continue
            else:
                prefix = line[:width]
                if b"+" in prefix and b"-" not in prefix:
                    self.scan_line("%s %s:%d" % (where, path, lineno), line[width:])
                if b"-" not in prefix:
                    lineno += 1


def check_commits(c, rev_range):
    for sha in git("rev-list", "--reverse", rev_range).decode().split():
        short = "commit %s" % sha[:12]
        parents = len(git("rev-list", "--parents", "-n1", sha).split()) - 1
        c.scan(short + " message", git("log", "-1", "--format=%B", sha))
        c.scan(short + " author/committer", git("log", "-1", "--format=%an <%ae>%n%cn <%ce>", sha))
        args = ["show", "--format=", "--no-color", "--no-ext-diff", "-U0", "-M"]
        if parents > 1:
            args.append("--cc")
        c.scan_patch(short, git(*args, sha), parents)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--commits")
    p.add_argument("--staged", action="store_true")
    p.add_argument("--message-file")
    p.add_argument("--env-text", action="append", default=[])
    p.add_argument("--require-list", action="store_true")
    args = p.parse_args(argv)

    patterns = load_patterns()
    if patterns is None:
        print("banned-word check: no list given (BANNED_WORDS_FILE or BANNED_WORDS)")
        return 1 if args.require_list else 0
    c = Checker(patterns)

    if args.commits:
        check_commits(c, args.commits)
    if args.staged:
        c.scan_patch("staged", git("diff", "--cached", "--no-color", "--no-ext-diff", "-U0", "-M"), 1)
    if args.message_file:
        with open(args.message_file, "rb") as f:
            c.scan("commit message", f.read())
    for var in args.env_text:
        c.scan(var, os.environ.get(var, "").encode("utf-8"))

    findings = list(dict.fromkeys(c.findings))
    for f in findings:
        print(f)
    if findings:
        print("banned-word check: %d finding(s)" % len(findings), file=sys.stderr)
        return 1
    print("banned-word check: clean (%d words)" % len(patterns))
    return 0


if __name__ == "__main__":
    sys.exit(main())
