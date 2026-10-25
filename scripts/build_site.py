#!/usr/bin/env python3
"""Build the project site with Zensical, in English and Japanese.

The Markdown under docs/ is written to read well on GitHub: pages link to
the READMEs and to code outside docs/. This script copies the pages into
build/site_src/<lang>/docs, makes each README the home page of its
language, and rewrites the links for the site:

- a link to a page or image of the same language stays relative;
- a link to the other language's page becomes an absolute site URL;
- a link to anything else in the repository points at GitHub.

The navigation follows each README's documentation list. Zensical builds
the English site into site/ and the Japanese site into site/ja/.

    pip install zensical
    python3 scripts/build_site.py           # build (--strict: fail on broken links)
    python3 scripts/build_site.py --serve   # build, then serve the English site
"""

import argparse
import posixpath
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REPO = "https://github.com/atinfinity/kachaka_isaac_sim"
SITE_URL = "https://atinfinity.github.io/kachaka_isaac_sim/"
STAGE = ROOT / "build" / "site_src"
SITE = ROOT / "site"

LANGS = {
    "en": {"readme": "README.md", "pages": "docs", "prefix": "", "name": "English",
           "home": "Home", "docs_heading": "## Documentation"},
    "ja": {"readme": "README.ja.md", "pages": "docs/ja", "prefix": "ja/", "name": "日本語",
           "home": "ホーム", "docs_heading": "## ドキュメント"},
}

LINK = re.compile(r"(!?\[[^\]]*\]\()([^)\s]+)(\))")
FENCE = re.compile(r"^\s*(```|~~~)")


def page_map():
    """Repository path of each page -> (language, path under docs_dir)."""
    pages = {}
    for lang, cfg in LANGS.items():
        pages[cfg["readme"]] = (lang, "index.md")
        for md in sorted((ROOT / cfg["pages"]).glob("*.md")):
            pages[md.relative_to(ROOT).as_posix()] = (lang, md.name)
    return pages


def site_url(lang, path):
    """Absolute URL of a staged page, with directory URLs."""
    stem = path[:-3]
    page = "" if stem == "index" else stem + "/"
    return SITE_URL + LANGS[lang]["prefix"] + page


def rewrite_target(target, src, lang, staged, pages):
    if re.match(r"^[a-z]+:|^#|^/", target):
        return target
    path, _, anchor = target.partition("#")
    anchor = "#" + anchor if anchor else ""
    repo_path = posixpath.normpath(posixpath.join(posixpath.dirname(src), path))
    if repo_path.startswith("docs/images/"):
        rel = posixpath.relpath(repo_path[len("docs/"):], posixpath.dirname(staged) or ".")
        return rel + anchor
    if repo_path in pages:
        to_lang, to_path = pages[repo_path]
        if to_lang == lang:
            return posixpath.relpath(to_path, posixpath.dirname(staged) or ".") + anchor
        return site_url(to_lang, to_path) + anchor
    kind = "tree" if (ROOT / repo_path).is_dir() else "blob"
    return f"{REPO}/{kind}/main/{repo_path}{anchor}"


def rewrite(text, src, lang, staged, pages):
    out, fenced = [], False
    for line in text.splitlines(keepends=True):
        if FENCE.match(line):
            fenced = not fenced
        if not fenced:
            line = LINK.sub(lambda m: m[1] + rewrite_target(m[2], src, lang, staged, pages) + m[3],
                            line)
        out.append(line)
    return "".join(out)


def nav(lang, pages):
    """The README's documentation list as Zensical navigation."""
    cfg = LANGS[lang]
    text = (ROOT / cfg["readme"]).read_text()
    section = text.split(cfg["docs_heading"], 1)[1].split("\n## ", 1)[0]
    entries = [f'  {{ "{cfg["home"]}" = "index.md" }},']
    for title, target in re.findall(r"^- \[([^\]]+)\]\(([^)]+)\)", section, re.MULTILINE):
        to_lang, to_path = pages[target]
        link = to_path if to_lang == lang else site_url(to_lang, to_path)
        entries.append(f'  {{ "{title}" = "{link}" }},')
    return "\n".join(entries)


def config(lang, pages):
    cfg = LANGS[lang]
    alternate = ",\n".join(
        f'  {{ name = "{c["name"]}", link = "{SITE_URL}{c["prefix"]}", lang = "{code}" }}'
        for code, c in LANGS.items())
    return f"""\
[project]
site_name = "kachaka_isaac_sim"
site_description = "A Kachaka simulator on NVIDIA Isaac Sim with the real robot's ROS 2 and kachaka-api interfaces"
site_url = "{SITE_URL}{cfg["prefix"]}"
repo_url = "{REPO}"
repo_name = "atinfinity/kachaka_isaac_sim"
copyright = "Apache License 2.0. An unofficial project, not affiliated with Preferred Robotics, Inc."
docs_dir = "docs"
site_dir = "site"
nav = [
{nav(lang, pages)}
]

[project.extra]
alternate = [
{alternate}
]

# Unicode slugs, as on GitHub, so that links to Japanese headings work.
[project.markdown_extensions.toc]
permalink = true
slugify = {{ object = "pymdownx.slugs.slugify", kwds = {{ case = "lower" }} }}

[project.markdown_extensions.tables]
[project.markdown_extensions.admonition]
[project.markdown_extensions.attr_list]
[project.markdown_extensions.md_in_html]
[project.markdown_extensions."pymdownx.highlight"]
[project.markdown_extensions."pymdownx.inlinehilite"]
[project.markdown_extensions."pymdownx.superfences"]

[project.theme]
language = "{lang}"
features = [
  "navigation.instant",
  "navigation.tracking",
  "navigation.top",
  "navigation.footer",
  "content.code.copy",
  "toc.follow",
]

[[project.theme.palette]]
media = "(prefers-color-scheme: light)"
scheme = "default"
toggle.icon = "lucide/sun"
toggle.name = "Switch to dark mode"

[[project.theme.palette]]
media = "(prefers-color-scheme: dark)"
scheme = "slate"
toggle.icon = "lucide/moon"
toggle.name = "Switch to light mode"
"""


def stage(pages):
    shutil.rmtree(STAGE, ignore_errors=True)
    for lang in LANGS:
        docs = STAGE / lang / "docs"
        docs.mkdir(parents=True)
        shutil.copytree(ROOT / "docs" / "images", docs / "images")
        (STAGE / lang / "zensical.toml").write_text(config(lang, pages))
    for src, (lang, staged) in pages.items():
        text = (ROOT / src).read_text()
        (STAGE / lang / "docs" / staged).write_text(rewrite(text, src, lang, staged, pages))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--serve", action="store_true", help="serve the English site after building")
    parser.add_argument("--strict", action="store_true", help="fail on warnings, such as broken links")
    args = parser.parse_args()
    pages = page_map()
    stage(pages)
    shutil.rmtree(SITE, ignore_errors=True)
    for lang in LANGS:
        strict = ["--strict"] if args.strict else []
        subprocess.run(["zensical", "build", "--clean", *strict], cwd=STAGE / lang, check=True)
        shutil.copytree(STAGE / lang / "site", SITE / LANGS[lang]["prefix"], dirs_exist_ok=True)
    print(f"[site] built {SITE}")
    if args.serve:
        subprocess.run(["zensical", "serve"], cwd=STAGE / "en", check=True)


if __name__ == "__main__":
    sys.exit(main())
