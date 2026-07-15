# Plan: MkDocs documentation site (theory → implementation)

**Goal:** Full documentation (getting started, theory-to-code bridge from the ICAPS 2018 paper, guides, API reference) built with Material for MkDocs, deployable as a static site to Cloudflare Pages; gif + paper stored via Git LFS and the gif featured in the README.

## Approach

Material for MkDocs + `mkdocstrings[python]` (API reference from docstrings) + `pymdownx.arithmatex` with KaTeX (LaTeX equations). Static `site/` output deploys to Cloudflare Pages. Because CF Pages' git integration does not fetch LFS objects, the site is built in a GitHub Action (`lfs: true` checkout) and deployed with `wrangler pages deploy` — never built by CF Pages itself.

Theory pages follow the paper (Pecora, Andreasson, Mansouri, Petkov — ICAPS 2018): each definition/equation/algorithm is stated in LaTeX, then immediately mapped to the class/function that implements it. No re-derivation, no paper prose — statement, code pointer, one-line note on where the port deviates (if it does).

## Changes

- `pyproject.toml` — add `docs` extra: `mkdocs-material`, `mkdocstrings[python]`.
- `.gitattributes` — new; LFS patterns for `*.gif` and `*.pdf`.
- `mkdocs.yml` — new; site config, nav, arithmatex + KaTeX, mkdocstrings.
- `docs/` — new tree (see structure below).
- `docs/assets/CoordinatorPy.gif`, `docs/assets/Paper.pdf` — moved from repo root (LFS-tracked); mkdocs bundles `docs/` assets into the site.
- `README.md` — gif at top, link to docs site, fix the stale Layout section (missing `motionplanning/`, `simulation2D/`, `viz/`; the "visualization out of scope" claim is outdated).
- `.github/workflows/docs.yml` — new; build + deploy to Cloudflare Pages on push to `master`.

### Docs structure

```
docs/
├── index.md                     # one-paragraph pitch, gif, paper citation + PDF link
├── getting-started.md           # install, run dynamic_missions demo (gif), viewer flags
├── theory/
│   ├── envelopes.md             # Defs 1–8, eqs (1)–(5): paths, spatial envelopes,
│   │                            #   critical sections, precedence constraints, critical points
│   │                            #   → metacsp/spatial, critical_section.py, dependency.py
│   ├── coordination.md          # Algorithms 1–2 (coordination loop, reviseConstraints),
│   │                            #   Lemma 1, Theorem 1
│   │                            #   → trajectory_envelope_coordinator.py, coordinator/
│   └── ordering-and-deadlocks.md# eqs (6)–(11): simple + heuristic ordering, forward
│                                #   models, h_dist; Def 9 unsafe cycles, Remark 3
│                                #   → forward_model.py, coordinator deadlock check
├── guides/
│   ├── motion-planning.md       # Hybrid A* + Reeds-Shepp, occupancy maps (ROS-style)
│   ├── simulation.md            # RK4 tracker, simulation2D, dispatching missions
│   └── visualization.md         # pyglet viewer, web viewer, --web-viewer/--headless flags
├── reference/                   # one page per subpackage, each a `::: module` block:
│   │                            #   coordinator, metacsp, motionplanning,
│   │                            #   simulation2D, viz, top-level modules
└── assets/                      # CoordinatorPy.gif, Paper.pdf (both LFS)
```

Getting-started demo section: `pip install coordination-oru[viz]` (or `-e .[viz]` + frontend build for source checkouts), then `python examples/dynamic_missions.py --web-viewer`, gif shown as "what you should see", note the point-and-click goal posting.

## Steps

- [x] `git lfs track "*.gif" "*.pdf"`; `git mv` gif + PDF to `docs/assets/`; verify with `git check-attr filter` that both resolve to `lfs` (files added *after* tracking, so no history rewrite needed).
- [x] Add `docs` extra to `pyproject.toml`.
- [x] Write `mkdocs.yml`: Material theme (dark/light toggle), nav as above, `pymdownx.arithmatex` (generic mode) + KaTeX in `extra_javascript`/`extra_css`, mkdocstrings with `show_source: false`, repo link.
- [x] Write `index.md` and `getting-started.md` (gif embedded from `assets/`).
- [x] Write the three theory pages from the paper. Pattern per concept: LaTeX statement → "In the code:" pointer (`coordination_oru/...` path + class/method) → deviation note if any. Verify every code pointer against the actual source while writing (read the referenced files then; don't guess symbol names).
- [x] Write the three guide pages (source: existing README content, example scripts, `motionplanning/`, `viz/`).
- [x] Write `reference/` pages (one `::: coordination_oru.<pkg>` block each); fix any docstrings that render broken, but don't mass-edit docstrings.
- [x] Update `README.md`: gif at top (`docs/assets/CoordinatorPy.gif` relative path — GitHub renders LFS images), docs link, corrected Layout section.
- [x] `mkdocs build --strict` passes locally; spot-check equations and API pages with `mkdocs serve`.
- [x] Add `.github/workflows/docs.yml`: checkout with `lfs: true` → `pip install -e .[docs]` → `mkdocs build --strict` → `cloudflare/wrangler-action` `pages deploy site --project-name=<name>`. Gate the deploy step on `secrets.CLOUDFLARE_API_TOKEN` being set so CI passes before the CF project exists.
- [x] One commit at the end (single commit for the whole plan).

## Edge cases & risks

- **CF Pages + LFS:** git-integrated CF Pages builds check out pointer files, not blobs. Mitigated by building in GH Actions with `lfs: true` and direct-upload deploy. Do not connect the repo to CF Pages' own build system.
- **KaTeX via CDN** in `extra_javascript` means equations need network to render. Acceptable for a hosted site; if offline docs matter later, vendor KaTeX into `docs/assets/`.
- **mkdocstrings coverage:** pages render whatever docstrings exist; sparse modules will look thin. That's honest — improve docstrings opportunistically, not as part of this plan.
- **README on PyPI:** PyPI's renderer won't resolve the relative gif path. Fine to ignore now; if it grates, switch README to an absolute `raw.githubusercontent.com` URL (GitHub serves LFS blobs through it).

## User setup (outside the repo)

Create the Cloudflare Pages project (direct-upload type) and add `CLOUDFLARE_API_TOKEN` + `CLOUDFLARE_ACCOUNT_ID` as GitHub Actions secrets. Everything else is in-repo.
