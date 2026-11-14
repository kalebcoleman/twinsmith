# Releasing twinsmith

A release is a `v<version>` tag (`v0.4.0`) on a commit of `main`. Pushing it runs
[`release.yml`](../.github/workflows/release.yml), which builds the host package in `mcp/` with uv, checks it
and uploads it to [PyPI](https://pypi.org/project/twinsmith/) through **trusted publishing**: PyPI trusts that
workflow, run in the `pypi` environment of `kalebcoleman/twinsmith`, and no token is stored anywhere. Nothing
else publishes: no other workflow and no other trigger.

Only the `twinsmith` package (the MCP server in `mcp/`) goes to PyPI. The simulator environment in the root
`pyproject.toml` (`twinsmith-sim`) and the Kit extensions in `exts/` are not published. The `twinsmith` name on npm
holds a 0.0.1 placeholder and stays that way: there is no npm package to publish.

## Where the version lives

In one place: `[project] version` in [`mcp/pyproject.toml`](../mcp/pyproject.toml). The server reads it back from the
installed package (`twinsmith --version`, `status`). A few files keep a copy: the root `pyproject.toml`, the project
entry in `mcp/uv.lock` and `uv.lock`, and the README badge. Never edit any of them by hand. Use the script:

```bash
python3 scripts/version.py                  # print the version
python3 scripts/version.py set 0.4.0        # write it to mcp/pyproject.toml and every copy
python3 scripts/version.py check            # fail if a copy differs (CI runs this on every push)
```

Versions are canonical [PEP 440](https://peps.python.org/pep-0440/): `0.4.0`, `0.4.1`, `0.5.0rc1`, `0.5.0.post1`. The tag is
`v` + the version (`v0.5.0rc1`), and anything else fails the release. pip and uv skip a pre-release unless asked
(`twinsmith==0.5.0rc1` or `--prerelease allow`), so an `rc` is a safe way to try the whole path first.

## Cutting a release

1. **Check what you are releasing.** On an up-to-date `main`:
   - CI is green for that commit, and the host suite passes locally:
     `cd mcp && uv run --locked --isolated --python 3.12 --extra test --extra isaaclab --extra scene-test pytest -q`
   - `python3 scripts/check_docs.py` and `python3 scripts/check_assets.py` pass, and
     [vendoring](VENDORING.md) and the LICENSE files still describe what ships.
   - Write release notes that **state the engine and the Isaac Sim build the release was qualified on**, with the Isaac Lab
     version if Lab was exercised. Take them from a recorded live run ([verification](VERIFICATION.md)), not from memory,
     and claim nothing stronger than that run shows ([target engine](COMPATIBILITY.md#target-engine)). If the live checks
     did not run, say so.
2. **Bump the version** on a branch and open a pull request:
   ```bash
   git switch -c release/v0.4.0 origin/main
   python3 scripts/version.py set 0.4.0
   (cd mcp && uv lock --check)               # the edited lock is current
   git commit -am "Release 0.4.0"
   git push -u origin release/v0.4.0          # then open a PR against main
   ```
   The PR's `Package build and metadata` job runs the release build and its checks on this commit, with the same
   pinned build backend and twine.
3. **Merge the PR, then tag the merged commit** (an annotated tag, on `main`):
   ```bash
   git fetch origin
   git log -1 --format='%H %s' origin/main   # the "Release 0.4.0" commit, or the merge that brought it in
   git tag -a v0.4.0 -m "twinsmith 0.4.0" origin/main
   git push origin v0.4.0
   ```
4. **Watch the run** (Actions → Release). The build job fails, and nothing is uploaded, if the tag is not
   `v<version>`, the tagged commit is not on `main`, a copy of the version differs, or the files do not build, check or
   install. If the `pypi` environment has a required reviewer, approve the deployment when the publish job waits for it.
5. **Check the result:** `uvx --from twinsmith==0.4.0 twinsmith --version` prints `twinsmith 0.4.0`. Publish the
   release notes: `gh release create v0.4.0 --verify-tag --notes-file <notes>`.

The client configuration for an installed release is `{"command": "uvx", "args": ["twinsmith"]}`, with `DISPLAY` and
`TWINSMITH_PORT` in its `env` ([client configuration](MCP-SERVER.md#4-client-configuration)).

### When it goes wrong

- **The build job failed:** nothing reached PyPI. Fix the cause on `main` and release again. A tag that never
  published can be deleted and re-created (`git push origin :refs/tags/v0.4.0`, `git tag -d v0.4.0`), or skipped in
  favour of the next patch version.
- **The publish job failed** (most likely on the first release: a publisher field or the environment name does not
  match, or the deployment was rejected): fix the setup, then *Re-run failed jobs* on the same run. It uploads the
  same checked files; re-tagging would build them again. If one file was uploaded and the other was not, the re-run
  stops at "File already exists": release the next patch version instead.
- **The upload succeeded and the release is bad:** a version on PyPI cannot be replaced, and a deleted file name can
  never be uploaded again. Yank it on PyPI (Manage → Releases → Options → Yank) and release the next patch version.

## One-time setup (Kaleb)

Do these once before the first tag. None of them stores a secret.

1. **PyPI trusted publisher.** The `twinsmith` project already exists (the 0.0.1 placeholder), so add the publisher to
   it rather than as a pending publisher: [pypi.org](https://pypi.org/manage/project/twinsmith/settings/publishing/) →
   *Manage* → *Publishing* → *Add a new publisher* → *GitHub*:

   | Field | Value |
   |---|---|
   | Owner | `kalebcoleman` |
   | Repository name | `twinsmith` |
   | Workflow name | `release.yml` |
   | Environment name | `pypi` |

   If an API token was made for the placeholder upload, revoke it under *Account settings* → *API tokens*.
2. **GitHub environment.** In the repository: *Settings* → *Environments* → *New environment* named `pypi`. Under
   *Deployment branches and tags* choose *Selected branches and tags* and add the tag pattern `v[0-9]*`. Optionally add
   yourself as a *Required reviewer*, so that every upload waits for a click. Add no secrets.
3. **Optional: protect release tags.** *Settings* → *Rules* → *Rulesets* → *New tag ruleset*, target `v[0-9]*`, restrict
   creation, update and deletion, and bypass for repository admins only.

The publisher names the workflow file. Renaming `release.yml` or the `pypi` environment breaks publishing until the
publisher is changed to match.

## How the workflows fit together

| File | Runs on | Does |
|---|---|---|
| [`build-dist.yml`](../.github/workflows/build-dist.yml) | called by the two below | Checks the version copies (and, for a release, the tag and that its commit is on `main`); `uv build --no-sources`; checks the file names; `twine check --strict`; installs the wheel in a clean environment and runs `twinsmith --version`. For a release it keeps the files as the `dist` artifact. |
| [`ci.yml`](../.github/workflows/ci.yml), job `package` | every push and pull request | `build-dist.yml` without a tag. Never publishes. |
| [`release.yml`](../.github/workflows/release.yml) | a pushed `v[0-9]*` tag only | `build-dist.yml` with the tag, then the `publish` job: the `pypi` environment, `id-token: write`, and `pypa/gh-action-pypi-publish` on those exact files. |

`mcp/tests/test_release.py` keeps this true: one tag trigger, OIDC and no secret in any workflow, and no publishing
step outside `release.yml`.

The publish action is pinned to a commit (v1.14.3), and `build-dist.yml` checks the files with the twine that
release bundles (7.0.0). To update the action, take the new release's commit from
[its releases](https://github.com/pypa/gh-action-pypi-publish/releases), read its twine pin in
`requirements/runtime.txt`, and change both in one pull request. The build backend is pinned too (`hatchling==1.33.0`
in `mcp/pyproject.toml`). It writes Metadata-Version 2.5, which twine 7 and PyPI accept; a newer hatchling may need a
newer twine, so move them together and let the PR's package job confirm it.
