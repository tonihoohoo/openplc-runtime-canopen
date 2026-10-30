#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Resolve and check a CANopen fork release, and write its notes.

The same file is used by the runtime and the editor fork. A release is a tag
``v<upstream base>-canopen.<N>`` on a commit of ``canopen``; the upstream base
is read from the fork's version file (the runtime's ``VERSION`` or the editor's
``package.json``).

  canopen_release.py resolve --base-file VERSION --event push --ref-name v4.3.1-canopen.1
  canopen_release.py resolve --base-file package.json --event workflow_dispatch --ref-name canopen

``resolve`` prints ``key=value`` lines (and appends them to ``$GITHUB_OUTPUT``):
``tag``, ``sha``, ``base`` and ``create_tag`` (``true`` when the workflow must
create the tag itself). With ``--notes-out`` it also writes the release notes.
It exits non-zero, with the reason, when the release must not be published;
``--dry-run`` reports those reasons as warnings instead.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

TAG_RE = re.compile(r"^v(\d+\.\d+\.\d+)-canopen\.(\d+)$")
BASE_RE = re.compile(r"^v?(\d+\.\d+\.\d+)$")
BRANCH = "canopen"


class ReleaseError(Exception):
    """The release must not be published."""


@dataclass
class Release:
    tag: str
    sha: str
    base: str
    create_tag: bool


def git(*args: str) -> str:
    return subprocess.run(["git", *args], check=True, capture_output=True, text=True).stdout.strip()


def git_ok(*args: str) -> bool:
    return (
        subprocess.run(["git", *args], check=False, capture_output=True, text=True).returncode == 0
    )


def read_base(path: Path) -> str:
    """The upstream release the fork is based on, as x.y.z."""
    text = path.read_text(encoding="utf-8")
    if path.name == "package.json":
        data = json.loads(text)
        raw = data.get("version", "") if isinstance(data, dict) else ""
        raw = raw if isinstance(raw, str) else ""
    else:
        raw = text.strip()
    match = BASE_RE.match(raw)
    if not match:
        raise ReleaseError(f"{path} does not hold a plain upstream version (found {raw!r})")
    return match.group(1)


def parse_tag(tag: str) -> Tuple[Tuple[int, int, int], int]:
    match = TAG_RE.match(tag)
    if not match:
        raise ReleaseError(f"{tag!r} is not a release tag (v<x.y.z>-canopen.<N>)")
    major, minor, patch = (int(part) for part in match.group(1).split("."))
    return (major, minor, patch), int(match.group(2))


def fork_tags() -> List[str]:
    tags = [t for t in git("tag", "--list", "v*-canopen.*").splitlines() if TAG_RE.match(t)]
    return sorted(tags, key=parse_tag)


def next_tag(base: str, tags: List[str]) -> str:
    numbers = [parse_tag(t)[1] for t in tags if TAG_RE.match(t).group(1) == base]
    return f"v{base}-canopen.{max(numbers, default=0) + 1}"


def workflow_passed(workflow: str, sha: str) -> bool:
    out = subprocess.run(
        [
            "gh",
            "run",
            "list",
            "--workflow",
            workflow,
            "--commit",
            sha,
            "--json",
            "conclusion",
            "--limit",
            "20",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    runs = json.loads(out)
    return isinstance(runs, list) and any(
        isinstance(run, dict) and run.get("conclusion") == "success" for run in runs
    )


def resolve(args: argparse.Namespace) -> Tuple[Release, List[str]]:
    base = read_base(Path(args.base_file))
    tags = fork_tags()
    problems: List[str] = []

    if args.event == "push":
        tag = args.ref_name
        tag_base = TAG_RE.match(tag).group(1) if TAG_RE.match(tag) else None
        parse_tag(tag)
        if tag_base != base:
            problems.append(f"{tag} names base {tag_base}, but {args.base_file} holds {base}")
        sha = git("rev-parse", f"{tag}^{{commit}}")
        create = False
    else:
        # A manual run, or a pull request testing the workflow (always a dry run).
        if args.ref_name != BRANCH:
            problems.append(f"releases are made from {BRANCH}, not {args.ref_name}")
        sha = git("rev-parse", "HEAD")
        if args.version:
            tag = args.version
            parse_tag(tag)
            if TAG_RE.match(tag).group(1) != base:
                problems.append(f"{tag} does not match the upstream base {base}")
            if tag in tags:
                problems.append(f"{tag} already exists; releases are never replaced")
        else:
            tag = next_tag(base, tags)
        create = True

    if not git_ok("merge-base", "--is-ancestor", sha, f"origin/{BRANCH}"):
        problems.append(f"{sha[:12]} is not on {BRANCH}")
    for workflow in args.require_workflow:
        if not workflow_passed(workflow, sha):
            problems.append(f"{workflow} has no successful run for {sha[:12]}")

    return Release(tag=tag, sha=sha, base=base, create_tag=create), problems


def previous_tag(release: Release) -> Optional[str]:
    earlier = [
        t
        for t in fork_tags()
        if t != release.tag
        and parse_tag(t) < parse_tag(release.tag)
        and git_ok("merge-base", "--is-ancestor", t, release.sha)
    ]
    return earlier[-1] if earlier else None


def notes(release: Release, args: argparse.Namespace) -> str:
    previous = previous_tag(release)
    since = previous if previous else "origin/main"
    changes = git(
        "log", "--first-parent", "--no-merges", "--format=- %s", f"{since}..{release.sha}"
    )
    repo = os.environ.get("GITHUB_REPOSITORY", "")
    lines = [
        f"Unofficial CANopen build of {args.product}, based on upstream release {release.base}.",
        "",
    ]
    if args.about.strip():
        lines += [args.about.strip(), ""]
    if args.notes.strip():
        lines += [args.notes.strip(), ""]
    lines += [
        f"## Changes since {previous or 'upstream ' + release.base}",
        "",
        changes or "- None",
        "",
    ]
    for note in args.note:
        lines += [note, ""]
    if repo:
        lines.append(f"Source code: https://github.com/{repo}/tree/{release.tag}")
    return "\n".join(lines).rstrip() + "\n"


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("resolve")
    p.add_argument("--base-file", required=True)
    p.add_argument("--event", required=True, choices=["push", "workflow_dispatch", "pull_request"])
    p.add_argument("--ref-name", required=True)
    p.add_argument("--version", default="")
    p.add_argument("--require-workflow", action="append", default=[])
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--product", default="OpenPLC")
    p.add_argument("--about", default="")
    p.add_argument("--notes", default="")
    p.add_argument("--note", action="append", default=[])
    p.add_argument("--notes-out")
    args = parser.parse_args(argv)

    try:
        release, problems = resolve(args)
    except (ReleaseError, subprocess.CalledProcessError) as exc:
        detail = exc.stderr.strip() if isinstance(exc, subprocess.CalledProcessError) else str(exc)
        print(f"::error::{detail}")
        return 1

    for problem in problems:
        print(f"::{'warning' if args.dry_run else 'error'}::{problem}")
    if problems and not args.dry_run:
        return 1

    if args.notes_out:
        Path(args.notes_out).write_text(notes(release, args), encoding="utf-8")

    outputs = {
        "tag": release.tag,
        "sha": release.sha,
        "base": release.base,
        "create_tag": "true" if release.create_tag else "false",
    }
    text = "".join(f"{key}={value}\n" for key, value in outputs.items())
    print(text, end="")
    github_output = os.environ.get("GITHUB_OUTPUT")
    if github_output:
        with open(github_output, "a", encoding="utf-8") as handle:
            handle.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
