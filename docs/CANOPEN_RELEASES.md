# CANopen releases

This fork publishes its own releases; upstream's release workflows (`docker.yml`,
`windows-installer.yml`) are disabled in the fork's Actions settings and stay unchanged in the
repository. Installing a release is described in [CANOPEN.md](CANOPEN.md#installing-a-release).

## Versions

A release is a tag `v<upstream version>-canopen.<N>` on a commit of `canopen`. The upstream version
is the one in `VERSION`, the upstream release `canopen` contains; N starts at 1 for each upstream
version. `v4.3.1-canopen.2` follows `v4.3.1-canopen.1`, and after upstream 4.3.2 is merged the next
release is `v4.3.2-canopen.1`.

- Never push a tag without `-canopen.`: it would sort above every fork release of that version.
- Never move, delete or reuse a tag. A broken release is followed by the next N.
- Images built by hand for testing use a commit hash (`v4.3.1-canopen.<hash>`) and are never tagged
  in the repository.
- `VERSION` stays upstream's, so a source install reports the upstream version; release images report
  their tag.

## Making a release

`.github/workflows/canopen-release.yml` does the work, started in one of two ways:

- **By hand**: Actions, **CANopen release**, **Run workflow** on `canopen`. Optional notes are shown
  above the list of changes; leave the version empty to get the next N. **Dry run** builds the images
  without creating a tag, pushing or releasing.
- **By tag**: `git tag v4.3.1-canopen.2 origin/canopen && git push origin v4.3.1-canopen.2`.

The workflow:

1. Refuses the release unless the commit is on `canopen`, the **CANopen** workflow passed on it,
   the tag matches `VERSION` and is new, and the notes pass the banned-word check.
2. Creates the tag (when started by hand).
3. Builds `linux/arm64` on an arm64 runner and `linux/amd64` on an x64 runner, with no emulation.
   Lely is built once per pinned commit in the image's base OS and cached; Docker layers use the
   GitHub Actions cache.
4. Joins the two images into `ghcr.io/<owner>/openplc-runtime:<tag>`, moves `latest` to it and checks
   that both architectures are there.
5. Creates the GitHub Release: the upstream version, the fork's commits since the previous release
   and a link to the source at the tag.

It needs no secrets beyond the workflow token and the `BANNED_WORDS` list.

## One-time setup

- Default branch `canopen`, so **Run workflow** finds the workflow.
- Disable `docker.yml` and `windows-installer.yml` in the Actions settings.
- A tag ruleset on `v*`: only the owner and GitHub Actions create tags; updates and deletions blocked.
- After the first release, make the `openplc-runtime` package public in its package settings and
  check it is linked to this repository.

## After an upstream release

1. Sync `main` and merge it into `canopen` in its own pull request.
2. Disable any new release or publish workflow upstream added.
3. Release with N back at 1 on the new upstream version.
