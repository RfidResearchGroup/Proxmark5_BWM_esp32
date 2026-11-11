#!/usr/bin/env python3
#-----------------------------------------------------------------------------
# Copyright (C) Proxmark3 contributors. See AUTHORS.md for details.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# See LICENSE.txt for the text of the license.
#-----------------------------------------------------------------------------
# Bundle a BWM release into a proxmark3 checkout.
#
# Downloads proxmark5_ble_wifi_module.bin from a GitHub release (or takes a
# local file), verifies it, then updates the bin, its sha512, the PM5 guide and
# CHANGELOG.md in proxmark3 and commits them. Never pushes.
#
# usage:
#   tools/bwm_bundle_pm3.py --tag V2 --pm3 ~/proxmark3
#   tools/bwm_bundle_pm3.py --file build/proxmark5_ble_wifi_module.bin --dry-run
#-----------------------------------------------------------------------------

import argparse
import hashlib
import json
import os
import re
import struct
import subprocess
import sys
import urllib.error
import urllib.request

REPO = "RfidResearchGroup/Proxmark5_BWM_esp32"
ASSET = "proxmark5_ble_wifi_module.bin"
PROJECT = "proxmark5_ble_wifi_module"

PM3_BIN = "client/resources/" + ASSET
PM3_SHA = "client/resources/proxmark5_ble_wifi_module.sha512.txt"
PM3_GUIDE = "doc/md/PM5_Start_Here/Getting_Started_With_PM5.md"
PM3_CHANGELOG = "CHANGELOG.md"

ESP_IMAGE_MAGIC = 0xE9
ESP_APP_DESC_MAGIC = 0xABCD5432
ESP_CHIP_ID_ESP32C2 = 0x000C
OTA_SLOT_SIZE = 0x1C0000          # app_a / app_b in partitions.csv

GUIDE_RE = re.compile(r"^(\[#\]\s+BWM fw version\.+ )(\S+)\s*$", re.M)
UNRELEASED = "## [unreleased][unreleased]"

# pm3 client prefixes, see PrintAndLogEx() in client/src/ui.c
USE_COLOR = sys.stdout.isatty() and "NO_COLOR" not in os.environ


def color(s, code):
    return f"\033[{code}m{s}\033[0m" if USE_COLOR else s


def red(s):
    return color(s, "31")


def green(s):
    return color(s, "32")


def yellow(s):
    return color(s, "33")


def cyan(s):
    return color(s, "36")


def success(msg):
    print(f"[{green('+')}] {msg}")


def info(msg):
    print(f"[{yellow('=')}] {msg}")


def hint(msg):
    print(f"[{yellow('?')}] {msg}")


def warning(msg):
    print(f"[{cyan('!')}] {msg}")


def die(msg):
    sys.stdout.flush()
    print(f"[{red('!!')}] {msg}", file=sys.stderr)
    sys.exit(1)


#-----------------------------------------------------------------------------
# GitHub
#-----------------------------------------------------------------------------
def http_get(url, accept=None):
    req = urllib.request.Request(url, headers={"User-Agent": "bwm-bundle-pm3"})
    if accept:
        req.add_header("Accept", accept)
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.read()
    except urllib.error.HTTPError as e:
        die(f"{url}: HTTP {e.code} {e.reason}")
    except urllib.error.URLError as e:
        die(f"{url}: {e.reason}")


def fetch_release(repo, tag):
    path = f"releases/tags/{tag}" if tag else "releases/latest"
    rel = json.loads(http_get(f"https://api.github.com/repos/{repo}/{path}",
                              "application/vnd.github+json"))
    if rel.get("draft"):
        die(f"release {rel.get('tag_name')} is a draft")
    asset = next((a for a in rel.get("assets", []) if a["name"] == ASSET), None)
    if asset is None:
        die(f"release {rel.get('tag_name')} has no {ASSET} (did the release build finish?)")
    info(f"release {yellow(rel['tag_name'])}: {asset['browser_download_url']}")
    data = http_get(asset["browser_download_url"])
    if len(data) != asset["size"]:
        die(f"size {len(data)} != asset size {asset['size']}")
    digest = asset.get("digest") or ""
    if digest.startswith("sha256:"):
        if hashlib.sha256(data).hexdigest() != digest[7:]:
            die("sha256 does not match the release asset digest")
        success(f"sha256 matches the GitHub asset digest ( {green('ok')} )")
    return rel["tag_name"], data


#-----------------------------------------------------------------------------
# Image checks
#-----------------------------------------------------------------------------
def cstr(b):
    return b.split(b"\0", 1)[0].decode("ascii", "replace")


def verify_image(data, expect_ver):
    """Check an ESP32-C2 app image, return the app descriptor fields."""
    if len(data) < 0x100:
        die("file too small for an ESP app image")
    if len(data) > OTA_SLOT_SIZE:
        die(f"{len(data)} bytes does not fit an OTA slot ({OTA_SLOT_SIZE})")

    # esp_image_header_t (24 bytes)
    magic, nseg = data[0], data[1]
    chip_id = struct.unpack_from("<H", data, 12)[0]
    hash_appended = data[23]
    if magic != ESP_IMAGE_MAGIC:
        die(f"image magic 0x{magic:02x}, expected 0x{ESP_IMAGE_MAGIC:02x}")
    if chip_id != ESP_CHIP_ID_ESP32C2:
        die(f"chip id 0x{chip_id:04x}, expected ESP32-C2 (0x{ESP_CHIP_ID_ESP32C2:04x})")

    # segments, checksum over segment data
    off = 24
    csum = 0xEF
    for i in range(nseg):
        if off + 8 > len(data):
            die(f"segment {i} header past end of file")
        _, seglen = struct.unpack_from("<II", data, off)
        off += 8
        if off + seglen > len(data):
            die(f"segment {i} data past end of file")
        for b in data[off:off + seglen]:
            csum ^= b
        off += seglen
    off += 15 - (off % 16)          # checksum sits in the last byte of a 16-byte block
    if off >= len(data) or data[off] != csum:
        die("image checksum mismatch")
    off += 1
    if hash_appended:
        if off + 32 != len(data):
            die(f"unexpected trailing data ({len(data) - off - 32} bytes)")
        if hashlib.sha256(data[:off]).digest() != data[off:off + 32]:
            die("image validation hash mismatch")
    elif off != len(data):
        die("unexpected trailing data")
    what = "checksum and validation hash" if hash_appended else "checksum"
    success(f"image {what} valid, {nseg} segments ( {green('ok')} )")

    # esp_app_desc_t right after the first segment header
    if struct.unpack_from("<I", data, 0x20)[0] != ESP_APP_DESC_MAGIC:
        die("app descriptor magic missing")
    desc = {
        "version": cstr(data[0x30:0x50]),
        "project": cstr(data[0x50:0x70]),
        "time": cstr(data[0x70:0x80]),
        "date": cstr(data[0x80:0x90]),
        "idf": cstr(data[0x90:0xB0]),
    }
    if desc["project"] != PROJECT:
        die(f"project name \"{desc['project']}\", expected \"{PROJECT}\"")
    if expect_ver and desc["version"] != expect_ver:
        die(f"app version \"{desc['version']}\" does not match tag \"{expect_ver}\"")
    if desc["version"].endswith("-dirty"):
        die(f"app version \"{desc['version']}\" was built from a dirty tree")
    success(f"app {desc['project']} {yellow(desc['version'])}, built {desc['date']} {desc['time']}, IDF {desc['idf']}")
    return desc


#-----------------------------------------------------------------------------
# proxmark3
#-----------------------------------------------------------------------------
def git(pm3, *args, capture=True):
    r = subprocess.run(["git", "-C", pm3, *args], text=True,
                       stdout=subprocess.PIPE if capture else None)
    if r.returncode != 0:
        die(f"git {' '.join(args)} failed")
    return r.stdout.strip() if capture else ""


def check_pm3(pm3):
    for p in (PM3_BIN, PM3_SHA, PM3_GUIDE, PM3_CHANGELOG):
        if not os.path.isfile(os.path.join(pm3, p)):
            die(f"{pm3}: {p} not found, not a proxmark3 checkout?")
    if git(pm3, "diff", "--cached", "--name-only"):
        die(f"{pm3}: index has staged changes, commit or unstage them first")
    dirty = git(pm3, "status", "--porcelain", "--", PM3_BIN, PM3_SHA, PM3_GUIDE, PM3_CHANGELOG)
    if dirty:
        die(f"{pm3}: uncommitted changes in files this tool writes:\n{dirty}")
    branch = git(pm3, "rev-parse", "--abbrev-ref", "HEAD")
    if branch != "master":
        warning(f"proxmark3 is on branch {branch}, not master")


def read(pm3, p):
    with open(os.path.join(pm3, p), encoding="utf-8") as f:
        return f.read()


def write(pm3, p, text):
    with open(os.path.join(pm3, p), "w", encoding="utf-8", newline="") as f:
        f.write(text)


def update_pm3(pm3, data, ver, entry):
    with open(os.path.join(pm3, PM3_BIN), "rb") as f:
        old = f.read()
    if old == data:
        die(f"{PM3_BIN} is already this image")

    with open(os.path.join(pm3, PM3_BIN), "wb") as f:
        f.write(data)
    write(pm3, PM3_SHA, f"{hashlib.sha512(data).hexdigest()}  {ASSET}\n")
    success(f"{PM3_BIN}: {len(old)} -> {len(data)} bytes, sha512 updated")

    guide = read(pm3, PM3_GUIDE)
    m = GUIDE_RE.search(guide)
    if m is None:
        die(f"{PM3_GUIDE}: \"BWM fw version\" line not found")
    write(pm3, PM3_GUIDE, GUIDE_RE.sub(lambda m: m.group(1) + ver, guide, count=1))
    success(f"{PM3_GUIDE}: {m.group(2)} -> {yellow(ver)}")

    log = read(pm3, PM3_CHANGELOG)
    if UNRELEASED not in log:
        die(f"{PM3_CHANGELOG}: \"{UNRELEASED}\" section not found")
    write(pm3, PM3_CHANGELOG, log.replace(UNRELEASED, f"{UNRELEASED}\n{entry}", 1))
    success(f"{PM3_CHANGELOG}: {entry}")


def main():
    ap = argparse.ArgumentParser(description="Bundle a BWM release into a proxmark3 checkout.")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--tag", help="release tag (default: latest release)")
    src.add_argument("--file", help="local image instead of a release download")
    ap.add_argument("--pm3", default=os.path.expanduser("~/proxmark3"), help="proxmark3 checkout (default: %(default)s)")
    ap.add_argument("--repo", default=REPO, help="GitHub repo (default: %(default)s)")
    ap.add_argument("--summary", default="", help="changelog text after the version, e.g. \"stability fixes\"")
    ap.add_argument("--author", help="changelog author (default: @<pm3 git user.name>)")
    ap.add_argument("--trailer", action="append", default=[], help="extra commit trailer line, repeatable")
    ap.add_argument("--no-commit", action="store_true", help="update the files but do not commit")
    ap.add_argument("--dry-run", action="store_true", help="download and verify only")
    args = ap.parse_args()

    pm3 = os.path.abspath(args.pm3)
    if not args.dry_run:
        check_pm3(pm3)

    if args.file:
        with open(args.file, "rb") as f:
            data = f.read()
        tag = None
        info(f"local file {args.file}")
    else:
        tag, data = fetch_release(args.repo, args.tag)

    desc = verify_image(data, tag)
    ver = desc["version"]
    if args.dry_run:
        success("dry run, proxmark3 not touched")
        return

    author = args.author or "@" + git(pm3, "config", "user.name")
    summary = f" - {args.summary}" if args.summary else ""
    entry = f"- Changed PM5 bundled BWM firmware to {ver}{summary} ({author})"
    update_pm3(pm3, data, ver, entry)

    paths = [PM3_BIN, PM3_SHA, PM3_GUIDE, PM3_CHANGELOG]
    if args.no_commit:
        success("files updated, not committed")
        return

    origin = f"Release asset from {args.repo} tag {tag}" if tag else f"Local build {os.path.basename(args.file)}"
    msg = (f"PM5: bundle BWM firmware {ver}\n\n"
           f"{origin}\n(built {desc['date']} {desc['time']}, ESP-IDF {desc['idf']}).\n")
    if args.trailer:
        msg += "\n" + "\n".join(args.trailer) + "\n"
    git(pm3, "add", "--", *paths)
    git(pm3, "commit", "-q", "-m", msg, "--", *paths)
    success(f"committed {git(pm3, 'log', '--oneline', '-1')}")
    hint(f"test on a PM5 with  `{yellow('hw bwm upgrade -f ' + os.path.join(pm3, PM3_BIN))}`")
    hint("then push proxmark3")


if __name__ == "__main__":
    main()
